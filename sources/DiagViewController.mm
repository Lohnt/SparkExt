#import "DiagViewController.h"
#import "MemRead.h"
#import "Il2Cpp.h"
#import "DiagLog.h"

#import <UIKit/UIKit.h>
#include <atomic>
#include <math.h>
#include <mach/mach.h>
#include <mach/vm_map.h>
#include <mach/vm_region.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_CANDIDATES 256
#define SCAN_CHUNK_SIZE (1u << 20)
#define NEAR_MISS_EXAMPLES 10
#define RAW_DUMP_LIMIT 10
#define CLASS_REGION_RADIUS 0x100000ULL
#define INSTANCE_DUMP_SIZE 0x148
#define HEAP_DUMP_LIMIT 3

/* Set from the main thread by the STOP button; polled by every loop. */
static std::atomic<bool> gStopRequested(false);

/* One K_1779 hit plus whether the K_7301 config link happens to hang off it.
   config_link is reported, never used as an acceptance gate. */
typedef struct {
    uint64_t v;
    uint8_t  config_link;
} cand_t;

/* Coverage counters for one scan pass. */
typedef struct {
    uint64_t start;
    uint64_t regions_visited;
    uint64_t regions_no_read;
    uint64_t regions_guard;
    uint64_t regions_reserved;
    uint64_t regions_read_failed;
    uint64_t bytes_scanned;
    uint64_t k1779_hits;
    uint64_t config_link_hits;
    uint64_t class_self_hits;
    double   elapsed;
} scan_report;

/* Rejection bucket: total count plus the first few offending addresses. */
typedef struct {
    uint64_t total;
    uint64_t ex[NEAR_MISS_EXAMPLES];
    int      count;
} nm_bucket;

static void nm_add(nm_bucket *b, uint64_t addr)
{
    b->total++;
    if (b->count < NEAR_MISS_EXAMPLES)
        b->ex[b->count++] = addr;
}

static uint64_t rd64(mach_port_t task, uint64_t addr)
{
    uint64_t v = 0;
    mr_read64(task, addr, &v);
    return v;
}

static uint32_t rd32(mach_port_t task, uint64_t addr)
{
    uint32_t v = 0;
    mr_read32(task, addr, &v);
    return v;
}

static uint8_t rd8(mach_port_t task, uint64_t addr)
{
    uint8_t v = 0;
    mr_read8(task, addr, &v);
    return v;
}

static uint64_t dist64(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

/* A position is plausible if it is finite and inside the playable world box. */
static bool diag_vec3_in_range(mr_vec3 v)
{
    const float limit = 100000.0f;
    if (!isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z))
        return false;
    return fabsf(v.x) <= limit && fabsf(v.y) <= limit && fabsf(v.z) <= limit;
}

/* Unity rotations are unit quaternions; a dead slot reads as all-zero. */
static bool diag_quat_unit_ok(const mr_quat *q)
{
    float len = sqrtf(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
    return isfinite(len) && len >= 0.9f && len <= 1.1f;
}

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

/* Returns the number of candidates found, or -1 if a stop was requested.
   Fills *rep with coverage statistics even on the stopped/early paths. */
static int scan_candidates(mach_port_t task, uint64_t k1779, uint64_t k7301,
                           uint64_t def_table, cand_t *out, int maxout,
                           scan_report *rep)
{
    int found = 0;
    vm_address_t addr = 0;

    memset(rep, 0, sizeof(*rep));
    rep->start = addr;

    CFAbsoluteTime t0 = CFAbsoluteTimeGetCurrent();

    uint8_t *chunk = (uint8_t *)malloc(SCAN_CHUNK_SIZE);
    if (!chunk) {
        rep->elapsed = CFAbsoluteTimeGetCurrent() - t0;
        return 0;
    }

    for (int guard = 0; guard < 1000000; guard++) {
        if (gStopRequested.load())
            goto stopped;

        vm_size_t size = 0;
        vm_region_basic_info_data_64_t info;
        mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
        mach_port_t objectName = MACH_PORT_NULL;

        memset(&info, 0, sizeof(info));
        kern_return_t kr = vm_region_64(task,
                                        &addr,
                                        &size,
                                        VM_REGION_BASIC_INFO_64,
                                        (vm_region_info_t)&info,
                                        &count,
                                        &objectName);
        if (objectName != MACH_PORT_NULL)
            mach_port_deallocate(mach_task_self(), objectName);
        if (kr != KERN_SUCCESS)
            break;

        rep->regions_visited++;

        if (!(info.protection & VM_PROT_READ)) {
            /* PROT_NONE == 0: reserved/unmapped slot; anything else exists but
               is not readable (e.g. write- or execute-only). */
            if (info.protection == 0)
                rep->regions_reserved++;
            else
                rep->regions_no_read++;
        } else if (size < 8) {
            rep->regions_guard++;
        } else if (found < maxout) {
            vm_address_t cursor = addr;
            vm_size_t remaining = size;
            int failures = 0;

            while (remaining >= 8 && found < maxout) {
                if (gStopRequested.load())
                    goto stopped;

                vm_size_t want = remaining < SCAN_CHUNK_SIZE ? remaining : SCAN_CHUNK_SIZE;
                if (mr_read(task, cursor, chunk, want)) {
                    failures = 0;
                    rep->bytes_scanned += want;
                    for (vm_size_t off = 0; off + 8 <= want; off += 8) {
                        if (gStopRequested.load())
                            goto stopped;

                        uint64_t value = 0;
                        memcpy(&value, chunk + off, sizeof(value));
                        if (value != k1779)
                            continue;

                        rep->k1779_hits++;

                        uint64_t candidate = cursor + off;

                        /* A word inside Il2CppClass metadata can contain the
                           class pointer itself; those are not game objects. */
                        if (rd64(task, candidate) == k1779 &&
                            (dist64(candidate, k1779) <= CLASS_REGION_RADIUS ||
                             dist64(candidate, def_table) <= CLASS_REGION_RADIUS)) {
                            rep->class_self_hits++;
                            continue;
                        }

                        uint64_t link = rd64(task, candidate + 0x18);
                        uint8_t cfg = (link != 0 && rd64(task, link) == k7301) ? 1 : 0;
                        if (cfg)
                            rep->config_link_hits++;

                        out[found].v = candidate;
                        out[found].config_link = cfg;
                        found++;
                        if (found >= maxout)
                            break;
                    }
                } else {
                    if (++failures >= 3) {
                        rep->regions_read_failed++;
                        break;
                    }
                }
                cursor += want;
                remaining -= want;
            }
        }

        vm_address_t next = addr + size;
        if (next <= addr)
            break;
        addr = next;
    }

    free(chunk);
    rep->elapsed = CFAbsoluteTimeGetCurrent() - t0;
    return found;

stopped:
    free(chunk);
    rep->elapsed = CFAbsoluteTimeGetCurrent() - t0;
    return -1;
}

@interface DiagViewController () {
    UITextView *_textView;
    UIButton *_rerunButton;
    UIButton *_rescanButton;
    UIButton *_stopButton;
    UIButton *_exitButton;
    BOOL _started;
    mach_port_t _task;
    pid_t _pid;
    dispatch_queue_t _workQueue;
}

- (void)appendLog:(NSString *)line;
- (void)logLine:(NSString *)fmt, ...;
- (void)vlogLine:(NSString *)fmt, ...;
- (void)logNearMiss:(NSString *)reason bucket:(const nm_bucket *)b;
- (void)dumpCandidateRaw:(mach_port_t)task v:(uint64_t)v index:(int)idx reason:(const char *)reason;
- (void)dumpInstance:(mach_port_t)task v:(uint64_t)v index:(int)idx;
- (void)followPointers:(mach_port_t)task v:(uint64_t)v;
- (void)reRun;
- (void)rescan;
- (void)stopScan;
- (void)doExit;
- (void)startRun;
- (void)attachLoop;
- (void)runDiagnostic;

@end

@implementation DiagViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    diag_log("diag: viewDidLoad");

    _task = MACH_PORT_NULL;
    _pid = 0;
    _workQueue = dispatch_queue_create("dev.cops.diag.work", DISPATCH_QUEUE_SERIAL);

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

    [self.view bringSubviewToFront:_rerunButton];
    [self.view bringSubviewToFront:_rescanButton];
    [self.view bringSubviewToFront:_stopButton];
    [self.view bringSubviewToFront:_exitButton];
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

/* Verbose file only: hop details, near-miss examples, reject dumps. */
- (void)vlogLine:(NSString *)fmt, ...
{
    va_list ap;
    va_start(ap, fmt);
    NSString *line = [[NSString alloc] initWithFormat:fmt arguments:ap];
    va_end(ap);

    diag_scan_log(line);
}

- (void)logNearMiss:(NSString *)reason bucket:(const nm_bucket *)b
{
    for (int i = 0; i < b->count; i++)
        [self vlogLine:@"  e.g. %@ 0x%llx", reason, b->ex[i]];
}

/* Raw field dump for rejected hits: verbose file only. */
- (void)dumpCandidateRaw:(mach_port_t)task v:(uint64_t)v index:(int)idx reason:(const char *)reason
{
    [self vlogLine:@"--- reject[%d] V=0x%llx  (%s) ---", idx, v, reason];
    [self vlogLine:@"  read64(V)  = 0x%llx", rd64(task, v)];

    static const uint64_t offs[] = { 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40 };
    for (int k = 0; k < 7; k++) {
        uint64_t p = 0;
        if (!mr_read64(task, v + offs[k], &p)) {
            [self vlogLine:@"  V+0x%02llx     = <unreadable>", offs[k]];
        } else if (p == 0) {
            [self vlogLine:@"  V+0x%02llx     = 0x0", offs[k]];
        } else {
            [self vlogLine:@"  V+0x%02llx     = 0x%llx -> read64 = 0x%llx",
                        offs[k], p, rd64(task, p)];
        }
    }
}

/* Ground truth: 0x148 bytes of the instance, 16 u64 per line, with the same
   words below as float32 (x y z w ...). Shown on screen and in the file. */
- (void)dumpInstance:(mach_port_t)task v:(uint64_t)v index:(int)idx
{
    unsigned char buf[INSTANCE_DUMP_SIZE];
    memset(buf, 0, sizeof(buf));

    if (!mr_read(task, v, buf, sizeof(buf))) {
        [self logLine:@"DUMP cand[%d] V=0x%llx <unreadable>", idx, v];
        return;
    }

    [self logLine:@"DUMP cand[%d] V=0x%llx", idx, v];

    for (size_t off = 0; off < INSTANCE_DUMP_SIZE; off += 128) {
        size_t avail = INSTANCE_DUMP_SIZE - off;
        size_t words = (avail < 128 ? avail : 128) / 8;

        NSMutableString *hexLine = [NSMutableString stringWithFormat:@"  +0x%03zx  ", off];
        NSMutableString *f32Line = [NSMutableString stringWithString:@"  f32       "];

        for (size_t w = 0; w < words; w++) {
            uint64_t word = 0;
            memcpy(&word, buf + off + w * 8, sizeof(word));
            [hexLine appendFormat:@"0x%016llx ", (unsigned long long)word];

            float f[2];
            memcpy(f, buf + off + w * 8, sizeof(f));
            if (w % 2 == 0)
                [f32Line appendString:@" "];
            [f32Line appendFormat:@"%10.3f %10.3f ", f[0], f[1]];
        }

        [self logLine:@"%@", hexLine];
        [self logLine:@"%@", f32Line];
    }
}

/* Every plausible pointer in the instance, with its first two qwords. */
- (void)followPointers:(mach_port_t)task v:(uint64_t)v
{
    [self logLine:@"FOLLOW ptrs:"];

    for (uint64_t off = 0x10; off <= 0x140; off += 8) {
        uint64_t p = 0;
        if (!mr_read64(task, v + off, &p))
            continue;
        if (p < 0x100000000ULL || (p & 0xF) != 0)
            continue;

        uint64_t q = rd64(task, p);
        uint64_t q2 = rd64(task, p + 8);
        [self logLine:@"V+0x%02llx -> 0x%llx -> [0x%llx, 0x%llx]",
                   (unsigned long long)off, p, q, q2];
    }
}

/* Main thread: cancel any in-flight work and queue a fresh run on the worker. */
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

/* RESCAN: same as RE-RUN, but reachable next to STOP and always instrumented. */
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
    [self logLine:@"stop requested; scan will abort at next checkpoint"];
}

/* Guaranteed escape hatch, independent of any queue or scan state. */
- (void)doExit
{
    diag_log("diag: exit pressed");
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

/* Worker queue: all reads and the heap scan happen off the main thread;
   every UI write goes through appendLog, which hops to the main queue. */
- (void)runDiagnostic
{
    mach_port_t task = _task;
    if (task == MACH_PORT_NULL) {
        [self logLine:@"no task port"];
        return;
    }

    if (gStopRequested.load()) {
        [self logLine:@"scan stopped"];
        return;
    }

    [self vlogLine:@"=== scan start ==="];

    il2cpp_ctx ctx;
    il2cpp_bootstrap(task, &ctx);
    diag_log("diag: bootstrap ok");

    [self logLine:@"uf_base = 0x%llx", ctx.uf_base];
    if (!ctx.uf_base) {
        [self logLine:@"FAIL: UnityFramework Mach-O not found"];
        return;
    }

    [self logLine:@"metadata_base = 0x%llx", ctx.metadata_base];
    [self logLine:@"name = %s", ctx.name];
    [self logLine:@"name_verify: %s", ctx.name_ok ? "PASS" : "FAIL"];
    [self logLine:@"def_table = 0x%llx", ctx.def_table];
    [self logLine:@"K_1779 = 0x%llx", ctx.k1779];
    [self logLine:@"K_7301 = 0x%llx", ctx.k7301];
    [self logLine:@"verify_name_ptr: %s", ctx.name_ptr_ok ? "PASS" : "FAIL"];
    [self logLine:@"verify_instance_size: %s", ctx.instance_size_ok ? "PASS" : "FAIL"];
    [self logLine:@"verify_token: %s", ctx.token_ok ? "PASS" : "FAIL"];

    if (!ctx.k1779) {
        [self logLine:@"FAIL: K_1779 unavailable"];
        return;
    }

    if (gStopRequested.load()) {
        [self logLine:@"scan stopped"];
        return;
    }

    [self logLine:@"scanning heap..."];

    cand_t candidates[MAX_CANDIDATES];
    bool acceptedFlags[MAX_CANDIDATES];
    memset(acceptedFlags, 0, sizeof(acceptedFlags));

    scan_report rep;
    CFAbsoluteTime tScan = CFAbsoluteTimeGetCurrent();
    int count = scan_candidates(task, ctx.k1779, ctx.k7301, ctx.def_table,
                                candidates, MAX_CANDIDATES, &rep);
    double scanWall = CFAbsoluteTimeGetCurrent() - tScan;
    diag_log("diag: scan returned");

    [self logLine:@"--- scan coverage ---"];
    [self logLine:@"  start address   = 0x%llx", rep.start];
    [self logLine:@"  regions visited = %llu", rep.regions_visited];
    [self logLine:@"  skipped no-read = %llu", rep.regions_no_read];
    [self logLine:@"  skipped guard   = %llu", rep.regions_guard];
    [self logLine:@"  skipped reserved= %llu", rep.regions_reserved];
    [self logLine:@"  read-failed     = %llu", rep.regions_read_failed];
    [self logLine:@"  bytes scanned   = %llu (%.1f MB)", rep.bytes_scanned,
               (double)rep.bytes_scanned / (1024.0 * 1024.0)];
    [self logLine:@"  elapsed         = %.2f s (wall %.2f s)", rep.elapsed, scanWall];
    [self logLine:@"  k1779 hits      = %llu (config_link hits %llu)",
               rep.k1779_hits, rep.config_link_hits];
    [self logLine:@"  class self-hits = %llu (excluded)", rep.class_self_hits];
    [self logLine:@"  candidates kept = %d (cap %d)", count < 0 ? 0 : count, MAX_CANDIDATES];

    if (count < 0) {
        diag_log("diag: scan aborted by stop");
        [self logLine:@"scan stopped"];
        return;
    }

    static const char *nmNames[2] = {
        "pos_a chain unresolved",
        "pos out of range",
    };
    nm_bucket nm[2];
    memset(nm, 0, sizeof(nm));

    int accepted = 0;
    int dumpedRejects = 0;

    for (int i = 0; i < count; i++) {
        if (gStopRequested.load()) {
            [self logLine:@"scan stopped"];
            return;
        }

        uint64_t v = candidates[i].v;

        /* Quat is informational: both paths are tried, the winner is reported,
           but a bad quat never rejects a candidate. */
        uint64_t quatPtr = 0;
        mr_quat quat;
        memset(&quat, 0, sizeof(quat));
        mr_quat dq;
        memset(&dq, 0, sizeof(dq));
        mr_quat iq;
        memset(&iq, 0, sizeof(iq));

        const char *qsrc = "none";
        bool derefRead = (mr_read64(task, v + 0xC0, &quatPtr) && quatPtr != 0) &&
                         mr_read_quat(task, quatPtr + 0x78, &dq);
        bool derefUnit = derefRead && diag_quat_unit_ok(&dq);

        bool inlineRead = mr_read_quat(task, v + 0xC0, &iq);
        bool inlineUnit = inlineRead && diag_quat_unit_ok(&iq);

        if (derefUnit) {
            quat = dq;
            qsrc = "deref";
        } else if (inlineUnit) {
            quat = iq;
            qsrc = "inline";
        } else if (derefRead) {
            quat = dq;
            qsrc = "deref(nonunit)";
        } else if (inlineRead) {
            quat = iq;
            qsrc = "inline(nonunit)";
        }

        float qlen = sqrtf(quat.x * quat.x + quat.y * quat.y +
                           quat.z * quat.z + quat.w * quat.w);

        /* pos_a: V+0x28 -> +0x18 -> vec3 at +0xD4. Every hop is logged. */
        uint64_t h0 = 0;
        bool h0ok = mr_read64(task, v + 0x28, &h0) && h0 != 0;

        uint64_t h1 = 0;
        bool h1ok = h0ok && mr_read64(task, h0 + 0x18, &h1) && h1 != 0;

        mr_vec3 posA;
        memset(&posA, 0, sizeof(posA));
        bool vecOk = h1ok && mr_read_vec3(task, h1 + 0xD4, &posA);
        bool rangeOk = vecOk && diag_vec3_in_range(posA);

        /* pos_b: V+0x30 -> +0x40 -> four vec3s. */
        uint64_t b = 0;
        bool bok = mr_read64(task, v + 0x30, &b) && b != 0;

        uint64_t bp = 0;
        bool bpok = bok && mr_read64(task, b + 0x40, &bp) && bp != 0;

        /* --- verbose detail, file only --- */
        [self vlogLine:@"cand[%d] %s @ 0x%llx config_link=%d", i,
                   rangeOk ? "ACCEPT" : "REJECT", v, (int)candidates[i].config_link];
        [self vlogLine:@"  read64(V)       = 0x%llx", rd64(task, v)];
        [self vlogLine:@"  read64(V+0xC0)  = 0x%llx", quatPtr];
        [self vlogLine:@"  quat            = (%.4f, %.4f, %.4f, %.4f) |q|=%.4f src=%s",
                    quat.x, quat.y, quat.z, quat.w, qlen, qsrc];
        [self vlogLine:@"  pos_a hop0      = read64(V+0x28)    = 0x%llx%s",
                    h0, h0ok ? "" : "  <failed>"];
        [self vlogLine:@"  pos_a hop1      = read64(hop0+0x18) = 0x%llx%s",
                    h1, h1ok ? "" : "  <failed>"];
        [self vlogLine:@"  pos_a xyz       = @hop1+0xD4        = (%.2f, %.2f, %.2f)%s",
                    posA.x, posA.y, posA.z, vecOk ? "" : "  <failed>"];
        [self vlogLine:@"  pos_b hop0      = read64(V+0x30)    = 0x%llx%s",
                    b, bok ? "" : "  <failed>"];
        [self vlogLine:@"  pos_b hop1      = read64(hop0+0x40) = 0x%llx%s",
                    bp, bpok ? "" : "  <failed>"];

        /* --- compact screen line --- */
        [self logLine:@"cand[%d] V=0x%llx cfg=%d |q|=%.4f pos_a=(%.2f, %.2f, %.2f)",
                   i, v, (int)candidates[i].config_link, qlen, posA.x, posA.y, posA.z];

        if (rangeOk) {
            accepted++;
            acceptedFlags[i] = true;

            mr_vec3 posB;
            memset(&posB, 0, sizeof(posB));
            bool b0ok = bpok && mr_read_vec3(task, bp + 0x10, &posB);
            [self vlogLine:@"  pos_b0 xyz      = @hop1+0x10        = (%.2f, %.2f, %.2f)%s",
                        posB.x, posB.y, posB.z, b0ok ? "" : "  <failed>"];

            memset(&posB, 0, sizeof(posB));
            bool b1ok = bpok && mr_read_vec3(task, bp + 0x1C, &posB);
            [self vlogLine:@"  pos_b1 xyz      = @hop1+0x1C        = (%.2f, %.2f, %.2f)%s",
                        posB.x, posB.y, posB.z, b1ok ? "" : "  <failed>"];

            memset(&posB, 0, sizeof(posB));
            bool b2ok = bpok && mr_read_vec3(task, bp + 0x2C, &posB);
            [self vlogLine:@"  pos_b2 xyz      = @hop1+0x2C        = (%.2f, %.2f, %.2f)%s",
                        posB.x, posB.y, posB.z, b2ok ? "" : "  <failed>"];

            memset(&posB, 0, sizeof(posB));
            bool b3ok = bpok && mr_read_vec3(task, bp + 0x38, &posB);
            [self vlogLine:@"  pos_b3 xyz      = @hop1+0x38        = (%.2f, %.2f, %.2f)%s",
                        posB.x, posB.y, posB.z, b3ok ? "" : "  <failed>"];

            [self vlogLine:@"  self            = %d", (int)rd8(task, v + 0x128)];
            [self vlogLine:@"  td5878 i0/i1/i2 = %d / %d / %d",
                        (int)rd32(task, b + 0x10), (int)rd32(task, b + 0x14),
                        (int)rd32(task, b + 0x18)];
        } else {
            int nb = vecOk ? 1 : 0;
            nm_add(&nm[nb], v);

            if (dumpedRejects < RAW_DUMP_LIMIT) {
                dumpedRejects++;
                [self dumpCandidateRaw:task v:v index:i reason:nmNames[nb]];
            }
        }
    }

    /* Near-miss counters on screen; the example addresses stay in the file. */
    [self logLine:@"rejected: %s = %llu", nmNames[0], nm[0].total];
    [self logLine:@"rejected: %s = %llu", nmNames[1], nm[1].total];
    for (int k = 0; k < 2; k++)
        [self logNearMiss:[NSString stringWithUTF8String:nmNames[k]] bucket:&nm[k]];

    /* Ground truth dumps: the config_link=1 instance (whatever its acceptance
       state), then up to three accepted heap instances. */
    int cfgDumped = 0;
    int heapDumps = 0;

    for (int i = 0; i < count; i++) {
        if (candidates[i].config_link && cfgDumped < 1) {
            cfgDumped = 1;
            [self dumpInstance:task v:candidates[i].v index:i];
            [self followPointers:task v:candidates[i].v];
        } else if (acceptedFlags[i] && candidates[i].v > ctx.uf_base && heapDumps < HEAP_DUMP_LIMIT) {
            heapDumps++;
            [self dumpInstance:task v:candidates[i].v index:i];
        }
    }

    [self logLine:@"scan complete: %d accepted / %d candidates", accepted, count];
    diag_log("diag: scan complete");
}

@end
