#ifndef DIAG_PRIVATE_H
#define DIAG_PRIVATE_H

#import "DiagViewController.h"
#import "EspDraw.h"

#include <atomic>
#include <mach/mach.h>

/* Set from the main thread by the STOP button; polled by every scan loop.
   Defined once in DiagViewController.mm. */
extern std::atomic<bool> gStopRequested;

/* Log plumbing and the run entry point, all implemented by the UI glue. The
   scan module formats and emits through these exactly as it did when the whole
   pipeline lived in one file. */
@interface DiagViewController (DiagUI)
- (void)appendLog:(NSString *)line;
- (void)logLine:(NSString *)fmt, ...;
- (void)vlogLine:(NSString *)fmt, ...;
- (void)startRun;
@end

/* Scan entry point, called by the glue and by the attach loop. */
@interface DiagViewController (Scan)
- (void)runDiagnostic;
@end

#endif /* DIAG_PRIVATE_H */
