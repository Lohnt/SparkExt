#import "AppDelegate.h"
#import "OverlayWindow.h"
#import "DiagViewController.h"
#import "SBSAccessibilityWindowHostingController.h"
#import <objc/runtime.h>
#import <objc/message.h>

@interface AppDelegate ()
@property (strong) OverlayWindow *window;
@end

@implementation AppDelegate

- (BOOL)application:(UIApplication *)app didFinishLaunchingWithOptions:(NSDictionary *)opts {
    DiagViewController *vc = [DiagViewController new];
    self.window = [[OverlayWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
    self.window.rootViewController = vc;
    self.window.windowLevel = 10000010.0;
    self.window.hidden = NO;
    [self.window makeKeyAndVisible];

    // register with SpringBoard so it floats above the game
    Class host = objc_getClass("SBSAccessibilityWindowHostingController");
    if (host) {
        id ctl = [[host alloc] init];
        unsigned int ctx = ((unsigned int(*)(id,SEL))objc_msgSend)(self.window, NSSelectorFromString(@"_contextId"));
        double lvl = self.window.windowLevel;
        SEL sel = NSSelectorFromString(@"registerWindowWithContextID:atLevel:");
        NSMethodSignature *sig = [NSMethodSignature signatureWithObjCTypes:"v@:Id"];
        NSInvocation *inv = [NSInvocation invocationWithMethodSignature:sig];
        [inv setTarget:ctl]; [inv setSelector:sel];
        [inv setArgument:&ctx atIndex:2]; [inv setArgument:&lvl atIndex:3];
        [inv invoke];
    }
    return YES;
}

@end
