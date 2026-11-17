#import <UIKit/UIKit.h>
#import "AppDelegate.h"
#import "DiagLog.h"

int main(int argc, char *argv[]) {
    diag_log("diag: main");
    @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
    }
}
