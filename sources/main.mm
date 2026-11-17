#import <UIKit/UIKit.h>
#import "AppDelegate.h"
#import "DiagLog.h"

/* Holds the breadcrumb trail left by the previous process; DiagViewController
   shows it as "=== previous run ===". nil if the file could not be read. */
NSString *gDiagPreviousRunLog = nil;

int main(int argc, char *argv[]) {
    @autoreleasepool {
        /* Before anything else: snapshot the trail from the last launch, so this
           run's own breadcrumbs cannot overwrite it before we copy it out. */
        gDiagPreviousRunLog = [NSString stringWithContentsOfFile:@DIAG_BOOT_LOG_PATH
                                                       encoding:NSUTF8StringEncoding
                                                          error:NULL];

        diag_log("diag: main");

        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
    }
}
