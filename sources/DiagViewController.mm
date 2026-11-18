#import "DiagViewController.h"
#import "DiagPrivate.h"
#import "MemRead.h"
#import "Il2Cpp.h"
#import "DiagLog.h"
#import "EspDraw.h"

#import <UIKit/UIKit.h>
#include <atomic>
#include <unistd.h>

/* Round 29: this file is the UI glue only - the window, the buttons, the text
   view, the STOP/RESCAN/EXIT plumbing and the attach loop. The scan pipeline
   lives in DiagScan.mm, the ESP snapshot/projection in EspDraw.mm and the
   system window hosting in OverlayWindow.mm. Log lines are unchanged. */

std::atomic<bool> gStopRequested(false);

/* Sleep in 0.25 s slices so STOP is honoured during the attach retry backoff.
   Returns true if a stop was requested while waiting. */
static bool diag_sleep_checked(double seconds)
{
    int slices = (int)(seconds * 4.0 + 0.5);
    for (int i = 0; i < slices; i++) {
        if (gStopRequested.load())
            return true;
        usleep(250000);
    }
    return gStopRequested.load();
}

@implementation DiagViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    diag_log("diag: viewDidLoad");

    _task = MACH_PORT_NULL;
    _pid = 0;
    _workQueue = dispatch_queue_create("dev.cops.diag.work", DISPATCH_QUEUE_SERIAL);
    _hitCache = [NSMutableDictionary dictionary];
    _viewSlots = [NSMutableDictionary dictionary];
    _localV = 0;
    _actorSlots = [NSMutableDictionary dictionary];
    _tableSlots = [NSMutableDictionary dictionary];
    _scanCount = 0;
    _actorBases = [NSMutableArray array];
    _actorRootN = 0;
    _planetRegs = NULL;
    _planetNRegs = 0;
    _moverHist = [NSMutableDictionary dictionary];
    _moverOrder = [NSMutableArray array];
    _selfPrev = [NSMutableDictionary dictionary];
    _moverWatch = [NSMutableArray array];
    _prevSelfP[0] = _prevSelfP[1] = _prevSelfP[2] = 0.0f;
    _prevSelfValid = NO;
    _humanPrev = [NSMutableSet set];

    /* Round 29 ESP state: the toggle and FOV index are UI-owned; the roots and
       camera live in the worker staging buffer until espCompleteScan publishes
       an immutable snapshot. */
    _espMode = NO;
    _espFovIdx = ESP_FOV_DEFAULT;
    _espMirror = [[NSUserDefaults standardUserDefaults] boolForKey:@"diag.esp.mirror"];
    _espStageN = 0;
    _espStageBind = 0;
    _espStageHaveCam = NO;
    _espStageCamSrc = 0;
    _espStageCamP[0] = _espStageCamP[1] = _espStageCamP[2] = 0.0f;
    _espStageCamF[0] = _espStageCamF[1] = _espStageCamF[2] = 0.0f;
    _espTimer = nil;

    self.view.backgroundColor = [UIColor blackColor];

    _textView = [[UITextView alloc] initWithFrame:self.view.bounds];
    _textView.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
    _textView.backgroundColor = [UIColor blackColor];
    _textView.textColor = [UIColor colorWithRed:0.2f green:1.0f blue:0.2f alpha:1.0f];
    UIFont *menlo = [UIFont fontWithName:@"Menlo" size:11.0];
    _textView.font = menlo ?: [UIFont monospacedSystemFontOfSize:11.0 weight:UIFontWeightRegular];
    _textView.editable = NO;
    _textView.selectable = NO;
    _textView.textContainerInset = UIEdgeInsetsMake(50.0, 6.0, 6.0, 6.0);
    [self.view addSubview:_textView];

    /* Show the trail from the previous process first; if it died before
       UIApplicationMain we still see how far the run before that got. */
    NSString *prev = gDiagPreviousRunLog;
    if (prev.length > 0) {
        _textView.text = [NSString stringWithFormat:@"=== previous run ===\n%@\n", prev];
    } else {
        _textView.text = @"=== previous run ===\n"
                         @"(no readable log at /var/mobile/Documents/diag_boot.log)\n";
    }

    /* Buttons are created AFTER the text view and pushed to the front, so they
       always sit above the scroll view and stay pinned to the window edges.
       Layout right-to-left: EXIT | STOP | RESCAN. */

    _rerunButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_rerunButton setTitle:@"RE-RUN" forState:UIControlStateNormal];
    [_rerunButton setTitleColor:[UIColor greenColor] forState:UIControlStateNormal];
    _rerunButton.frame = CGRectMake(10.0, 18.0, 80.0, 30.0);
    _rerunButton.autoresizingMask = UIViewAutoresizingFlexibleRightMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_rerunButton addTarget:self action:@selector(reRun) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_rerunButton];

    _exitButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_exitButton setTitle:@"EXIT" forState:UIControlStateNormal];
    [_exitButton setTitleColor:[UIColor redColor] forState:UIControlStateNormal];
    _exitButton.frame = CGRectMake(self.view.bounds.size.width - 90.0, 18.0, 80.0, 30.0);
    _exitButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_exitButton addTarget:self action:@selector(doExit) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_exitButton];

    _stopButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_stopButton setTitle:@"STOP" forState:UIControlStateNormal];
    [_stopButton setTitleColor:[UIColor redColor] forState:UIControlStateNormal];
    _stopButton.frame = CGRectMake(self.view.bounds.size.width - 180.0, 18.0, 80.0, 30.0);
    _stopButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_stopButton addTarget:self action:@selector(stopScan) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_stopButton];

    _rescanButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_rescanButton setTitle:@"RESCAN" forState:UIControlStateNormal];
    [_rescanButton setTitleColor:[UIColor yellowColor] forState:UIControlStateNormal];
    _rescanButton.frame = CGRectMake(self.view.bounds.size.width - 270.0, 18.0, 80.0, 30.0);
    _rescanButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_rescanButton addTarget:self action:@selector(rescan) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_rescanButton];

    /* Round 28: ESP RESCAN sits under RE-RUN and runs the whole existing
       pipeline; the FOV control sits under RESCAN and only redraws, using the
       camera and humanoid roots the last scan cached. */
    _espButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_espButton setTitle:@"ESP RESCAN" forState:UIControlStateNormal];
    [_espButton setTitleColor:[UIColor cyanColor] forState:UIControlStateNormal];
    _espButton.frame = CGRectMake(10.0, 52.0, 110.0, 30.0);
    _espButton.autoresizingMask = UIViewAutoresizingFlexibleRightMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_espButton addTarget:self action:@selector(espRescan) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_espButton];

    NSArray *fovTitles = @[ @"40", @"60", @"80", @"90", @"100", @"110", @"120", @"130" ];
    _fovControl = [[UISegmentedControl alloc] initWithItems:fovTitles];
    _fovControl.frame = CGRectMake(self.view.bounds.size.width - 370.0, 52.0, 350.0, 28.0);
    _fovControl.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    _fovControl.selectedSegmentIndex = ESP_FOV_DEFAULT;
    [_fovControl addTarget:self action:@selector(fovChanged:)
         forControlEvents:UIControlEventValueChanged];
    [self.view addSubview:_fovControl];

    /* Round 32: horizontal mirror. The projection right vector is known to be
       handed the wrong way on some devices, so this flips the projected x about
       the overlay centre. Persisted like the rest of the ESP state. */
    _mirrorButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_mirrorButton setTitleColor:[UIColor cyanColor] forState:UIControlStateNormal];
    _mirrorButton.frame = CGRectMake(self.view.bounds.size.width - 470.0, 52.0, 95.0, 30.0);
    _mirrorButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_mirrorButton addTarget:self action:@selector(mirrorToggled:) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_mirrorButton];
    [self espUpdateMirrorTitle];

    [self.view bringSubviewToFront:_rerunButton];
    [self.view bringSubviewToFront:_espButton];
    [self.view bringSubviewToFront:_rescanButton];
    [self.view bringSubviewToFront:_fovControl];
    [self.view bringSubviewToFront:_mirrorButton];
    [self.view bringSubviewToFront:_stopButton];
    [self.view bringSubviewToFront:_exitButton];

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(espRotated:)
                                                 name:UIApplicationDidChangeStatusBarOrientationNotification
                                               object:nil];
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    if (!_started) {
        _started = YES;
        [self startRun];
    }
}

- (void)appendLog:(NSString *)line
{
    if (![NSThread isMainThread]) {
        __weak typeof(self) weakSelf = self;
        dispatch_async(dispatch_get_main_queue(), ^{
            [weakSelf appendLog:line];
        });
        return;
    }

    NSString *existing = _textView.text ?: @"";
    _textView.text = [existing stringByAppendingFormat:@"%@\n", line];
    if (_textView.text.length > 0)
        [_textView scrollRangeToVisible:NSMakeRange(_textView.text.length - 1, 1)];
}

/* Screen and verbose file. */
- (void)logLine:(NSString *)fmt, ...
{
    va_list ap;
    va_start(ap, fmt);
    NSString *line = [[NSString alloc] initWithFormat:fmt arguments:ap];
    va_end(ap);

    diag_scan_log(line);
    [self appendLog:line];
}

/* Verbose file only: discovery detail, chains, per-candidate hits. */
- (void)vlogLine:(NSString *)fmt, ...
{
    va_list ap;
    va_start(ap, fmt);
    NSString *line = [[NSString alloc] initWithFormat:fmt arguments:ap];
    va_end(ap);

    diag_scan_log(line);
}

- (void)startRun
{
    gStopRequested.store(true);

    __weak typeof(self) weakSelf = self;
    dispatch_async(_workQueue, ^{
        diag_log("diag: worker start");

        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf)
            return;

        gStopRequested.store(false);

        if (strongSelf->_task != MACH_PORT_NULL) {
            [strongSelf runDiagnostic];
        } else {
            [strongSelf attachLoop];
        }
    });
}

- (void)reRun
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self reRun]; });
        return;
    }

    _textView.text = @"";
    [self startRun];
}

/* RESCAN: same as RE-RUN, but reachable next to STOP and always instrumented.
   The hit cache survives, so the second pass reports SAME/CHANGED. */
- (void)rescan
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self rescan]; });
        return;
    }

    diag_log("diag: rescan pressed");
    _textView.text = @"";
    [self logLine:@"rescan from worker queue"];
    [self startRun];
}

- (void)stopScan
{
    diag_log("diag: stop pressed");
    gStopRequested.store(true);

    /* Round 29: STOP drops the ESP snapshot and hides the overlay. The scan
       threads only ever write the staging buffer, so there is nothing to
       drain here. */
    [self espSnapshotClear];

    [self logLine:@"stop requested; scan will abort at next checkpoint"];
}

/* Guaranteed escape hatch, independent of any queue or scan state. */
- (void)doExit
{
    diag_log("diag: exit pressed");

    /* Round 29: same snapshot drop before the process goes away. This is the
       main thread, so espSnapshotClear does not defer. */
    [self espSnapshotClear];

    exit(0);
}

/* Worker queue: find and attach to Critical Ops, at most 30 attempts. */
- (void)attachLoop
{
    diag_log("diag: attaching");
    [self logLine:@"looking for CriticalOps..."];

    int lastRc = 0;

    for (int attempt = 0; attempt < 30; attempt++) {
        if (gStopRequested.load()) {
            diag_log("diag: attach aborted by stop");
            [self logLine:@"scan stopped"];
            return;
        }

        pid_t pid = mr_find_pid_by_name("CriticalOps");
        if (pid > 0) {
            mach_port_t task = MACH_PORT_NULL;
            int rc = mr_attach(pid, &task);
            lastRc = rc;
            if (rc == 0) {
                _pid = pid;
                _task = task;
                [self logLine:@"attached pid=%d", pid];
                [self runDiagnostic];
                return;
            }
        }

        if (diag_sleep_checked(2.0)) {
            diag_log("diag: attach aborted by stop");
            [self logLine:@"scan stopped"];
            return;
        }
    }

    [self logLine:@"attach failed rc=%d", lastRc];
}


@end
