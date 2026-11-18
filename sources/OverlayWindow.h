#import <UIKit/UIKit.h>

/* Round 28: the ESP surface. One window above everything, hosted the way the
   local TrollSpeed external HUD hosts its system window (see +shared), and
   completely non-interactive: hitTest returns nil, so every touch the user
   makes lands in the game underneath. */
@interface OverlayWindow : UIWindow

/* Lazily builds the window, its transparent root view and registers it with
   SpringBoard's accessibility window hosting controller. Safe to call from any
   thread; the window itself is only ever touched on the main thread. */
+ (instancetype)shared;

/* Visibility. The window starts hidden; ESP RESCAN shows it. */
- (void)espShow:(BOOL)show;

/* Remove every box and label, keep the status text. */
- (void)espClear;

/* Replace the whole overlay in one pass: count boxes (screen points), one
   distance per box in metres, the top-left status line and the FOV tag.
   Main thread only. */
- (void)espSetRects:(const CGRect *)rects
          distances:(const float *)dists
              count:(int)count
             status:(NSString *)status
                fov:(NSString *)fov;

@end
