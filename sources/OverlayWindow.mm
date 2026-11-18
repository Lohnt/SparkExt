#import "OverlayWindow.h"
#import "DiagLog.h"
#import <objc/runtime.h>
#import <objc/message.h>

/* Round 28 ESP v1. Hosting approach copied from the local trollspeed-nullext
   build (README of that project: an accessibility window hosted by SpringBoard
   with SBSAccessibilityWindowHostingController). That build is a read-only
   reference; nothing from it is used here beyond the public runtime selectors.

   Drawing is deliberately dumb: one CAShapeLayer holds every box as a single
   path, labels are pooled UILabels. A redraw therefore costs one path rebuild
   no matter how many boxes there are, which is what makes the FOV segment
   control redraw instantly from the cached scan. */

static const CGFloat kEspStrokeWidth = 1.5;

static UIColor *esp_green(void)
{
    return [UIColor colorWithRed:0.20f green:1.00f blue:0.35f alpha:1.0f];
}

static UIFont *esp_font(CGFloat size, BOOL bold)
{
    NSString *name = bold ? @"Menlo-Bold" : @"Menlo";
    UIFont *f = [UIFont fontWithName:name size:size];
    if (f)
        return f;
    return [UIFont monospacedSystemFontOfSize:size
                                       weight:bold ? UIFontWeightBold : UIFontWeightRegular];
}

#pragma mark - transparent, non-interactive root view

@interface ESPOverlayView : UIView
@property (nonatomic, strong) CAShapeLayer *boxLayer;
@property (nonatomic, strong) NSMutableArray<UILabel *> *distLabels;
@property (nonatomic, strong) UILabel *statusLabel;
@property (nonatomic, strong) UILabel *fovLabel;
- (void)applyRects:(const CGRect *)rects
         distances:(const float *)dists
             count:(int)count
            status:(NSString *)status
               fov:(NSString *)fov;
@end

@implementation ESPOverlayView

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (!self)
        return nil;

    self.backgroundColor = [UIColor clearColor];
    self.opaque = NO;
    self.userInteractionEnabled = NO;

    _boxLayer = [CAShapeLayer layer];
    _boxLayer.frame = self.bounds;
    _boxLayer.fillColor = [UIColor clearColor].CGColor;
    _boxLayer.strokeColor = [esp_green() CGColor];
    _boxLayer.lineWidth = kEspStrokeWidth;
    _boxLayer.contentsScale = [UIScreen mainScreen].scale;
    [self.layer addSublayer:_boxLayer];

    _distLabels = [NSMutableArray array];

    _statusLabel = [[UILabel alloc] initWithFrame:CGRectZero];
    _statusLabel.font = esp_font(12.0, YES);
    _statusLabel.textColor = esp_green();
    _statusLabel.backgroundColor = [UIColor colorWithWhite:0.0 alpha:0.45];
    _statusLabel.textAlignment = NSTextAlignmentLeft;
    _statusLabel.numberOfLines = 1;
    [self addSubview:_statusLabel];

    _fovLabel = [[UILabel alloc] initWithFrame:CGRectZero];
    _fovLabel.font = esp_font(15.0, YES);
    _fovLabel.textColor = esp_green();
    _fovLabel.backgroundColor = [UIColor colorWithWhite:0.0 alpha:0.35];
    _fovLabel.textAlignment = NSTextAlignmentLeft;
    [self addSubview:_fovLabel];

    return self;
}

- (void)layoutSubviews
{
    [super layoutSubviews];

    self.boxLayer.frame = self.bounds;

    /* Top-left of the usable area: in landscape the notch sits on the left, so
       the text starts after the safe-area inset. */
    CGFloat x = self.safeAreaInsets.left + 8.0;
    CGFloat y = self.safeAreaInsets.top + 4.0;
    self.statusLabel.frame = CGRectMake(x, y, 470.0, 18.0);
    self.fovLabel.frame = CGRectMake(x, y + 19.0, 200.0, 20.0);
}

- (void)applyRects:(const CGRect *)rects
         distances:(const float *)dists
             count:(int)count
            status:(NSString *)status
               fov:(NSString *)fov
{
    if (count > 0 && rects) {
        UIBezierPath *path = [UIBezierPath bezierPath];
        for (int i = 0; i < count; i++) {
            if (rects[i].size.width < 1.0 || rects[i].size.height < 1.0)
                continue;
            [path appendPath:[UIBezierPath bezierPathWithRect:rects[i]]];
        }
        self.boxLayer.path = path.CGPath;
    } else {
        self.boxLayer.path = NULL;
    }

    while ((int)self.distLabels.count < count) {
        UILabel *l = [[UILabel alloc] initWithFrame:CGRectZero];
        l.font = esp_font(10.0, YES);
        l.textColor = esp_green();
        l.backgroundColor = [UIColor colorWithWhite:0.0 alpha:0.40];
        l.textAlignment = NSTextAlignmentCenter;
        l.hidden = YES;
        [self addSubview:l];
        [self.distLabels addObject:l];
    }

    for (int i = 0; i < (int)self.distLabels.count; i++) {
        UILabel *l = self.distLabels[i];
        if (i >= count || !dists) {
            l.hidden = YES;
            continue;
        }

        l.hidden = NO;
        l.text = [NSString stringWithFormat:@"%.0fm", (double)dists[i]];
        [l sizeToFit];

        CGRect b = rects[i];
        CGRect f = l.frame;
        f.origin.x = CGRectGetMidX(b) - f.size.width * 0.5f;
        f.origin.y = CGRectGetMinY(b) - f.size.height - 1.0f;
        if (f.origin.y < 0.0f)
            f.origin.y = CGRectGetMaxY(b) + 1.0f;
        l.frame = CGRectIntegral(f);
    }

    self.statusLabel.text = status;
    self.fovLabel.text = fov;
}

@end

#pragma mark - root view controller

@interface ESPOverlayController : UIViewController
@end

@implementation ESPOverlayController

- (void)loadView
{
    ESPOverlayView *v = [[ESPOverlayView alloc] initWithFrame:[UIScreen mainScreen].bounds];
    self.view = v;
}

- (BOOL)prefersStatusBarHidden { return YES; }
- (BOOL)shouldAutorotate { return YES; }
- (UIInterfaceOrientationMask)supportedInterfaceOrientations {
    return UIInterfaceOrientationMaskLandscape;
}
- (BOOL)prefersHomeIndicatorAutoHidden { return YES; }

@end

#pragma mark - window

@interface OverlayWindow ()
@property (nonatomic, strong) id hostingController;
- (ESPOverlayView *)espView;
- (void)hostWithSpringBoard;
@end

@implementation OverlayWindow

+ (instancetype)shared
{
    static OverlayWindow *window = nil;
    static dispatch_once_t once;

    dispatch_once(&once, ^{
        window = [[OverlayWindow alloc] initWithFrame:[UIScreen mainScreen].bounds];
        window.windowLevel = 10000020.0;
        window.backgroundColor = [UIColor clearColor];
        window.opaque = NO;
        window.userInteractionEnabled = NO;
        window.hidden = YES;
        window.rootViewController = [[ESPOverlayController alloc] init];
        [window hostWithSpringBoard];
        diag_log("diag: overlay window created");
    });

    return window;
}

/* Window-server flags: a system window that the window server hosts, exactly
   the shape the local TrollSpeed external HUD uses so SpringBoard keeps it
   composited above the game. */
+ (BOOL)_isSystemWindow { return YES; }
- (BOOL)_isWindowServerHostingManaged { return NO; }
- (BOOL)_isSecure { return YES; }
- (BOOL)_shouldCreateContextAsSecure { return YES; }

/* Touches must reach the game, never this window. */
- (BOOL)canBecomeKeyWindow { return NO; }
- (UIView *)hitTest:(CGPoint)point withEvent:(UIEvent *)event { return nil; }

- (ESPOverlayView *)espView
{
    return (ESPOverlayView *)self.rootViewController.view;
}

- (void)espShow:(BOOL)show
{
    self.hidden = !show;
    self.userInteractionEnabled = NO;
}

- (void)espClear
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self espClear]; });
        return;
    }

    [self espView].boxLayer.path = NULL;
    for (UILabel *l in [self espView].distLabels)
        l.hidden = YES;
}

- (void)espSetRects:(const CGRect *)rects
          distances:(const float *)dists
              count:(int)count
             status:(NSString *)status
                fov:(NSString *)fov
{
    if (![NSThread isMainThread]) {
        NSData *rb = [NSData dataWithBytes:rects length:(NSUInteger)count * sizeof(CGRect)];
        NSData *db = [NSData dataWithBytes:dists length:(NSUInteger)count * sizeof(float)];
        NSString *st = [status copy];
        NSString *fv = [fov copy];

        dispatch_async(dispatch_get_main_queue(), ^{
            [self espSetRects:(const CGRect *)rb.bytes
                    distances:(const float *)db.bytes
                        count:count
                       status:st
                          fov:fv];
        });
        return;
    }

    [[self espView] applyRects:rects distances:dists count:count status:status fov:fov];
}

/* SpringBoard registration. SBSAccessibilityWindowHostingController is a
   private class; every call is guarded, and the controller is retained so the
   registration cannot be torn down while the window is alive. */
- (void)hostWithSpringBoard
{
    Class hostCls = NSClassFromString(@"SBSAccessibilityWindowHostingController");
    if (!hostCls) {
        diag_log("diag: overlay: SBSAccessibilityWindowHostingController not found");
        return;
    }

    id ctl = [[hostCls alloc] init];
    if (!ctl) {
        diag_log("diag: overlay: hosting controller alloc failed");
        return;
    }

    if ([ctl respondsToSelector:@selector(setWindow:)]) {
        ((void (*)(id, SEL, id))objc_msgSend)(ctl, @selector(setWindow:), self);
    }

    SEL ctxSel = NSSelectorFromString(@"_contextId");
    unsigned ctx = 0;

    if ([self respondsToSelector:ctxSel])
        ctx = ((unsigned (*)(id, SEL))objc_msgSend)(self, ctxSel);

    if (!ctx) {
        UIWindow *key = [UIApplication sharedApplication].keyWindow;
        if (key && [key respondsToSelector:ctxSel])
            ctx = ((unsigned (*)(id, SEL))objc_msgSend)(key, ctxSel);
    }

    if (!ctx && [ctl respondsToSelector:ctxSel])
        ctx = ((unsigned (*)(id, SEL))objc_msgSend)(ctl, ctxSel);

    SEL reg = NSSelectorFromString(@"registerWindowWithContextID:atLevel:");
    if (![ctl respondsToSelector:reg]) {
        diag_log("diag: overlay: register selector missing");
        return;
    }

    double level = (double)self.windowLevel;
    ((void (*)(id, SEL, unsigned, double))objc_msgSend)(ctl, reg, ctx, level);
    self.hostingController = ctl;

    char line[192];
    snprintf(line, sizeof(line),
             "diag: overlay hosted ctx=%u level=%.0f", ctx, level);
    diag_log(line);
}

@end
