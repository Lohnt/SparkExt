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

/* Set from the main thread by the STOP button; polled by every loop. */
static std::atomic<bool> gStopRequested(false);

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
    uint64_t link_rejected;
    uint64_t link_examples[NEAR_MISS_EXAMPLES];
    int      link_example_count;
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

/* A position is plausible if it is finite and inside the playable world box. */
static bool diag_vec3_in_range(mr_vec3 v)
{
    const float limit = 100000.0f;
    if (!isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z))
        return false;
    return fabsf(v.x) <= limit && fabsf(v.y) <= limit && fabsf(v.z) <= limit;
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
                           uint64_t *out, int maxout, scan_report *rep)
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
                        uint64_t link = rd64(task, candidate + 0x18);
                        if (link != 0 && rd64(task, link) == k7301) {
                            out[found++] = candidate;
                            if (found >= maxout)
                                break;
                        } else {
                            rep->link_rejected++;
                            if (rep->link_example_count < NEAR_MISS_EXAMPLES)
                                rep->link_examples[rep->link_example_count++] = candidate;
                        }
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
    BOOL _started;
    mach_port_t _task;
    pid_t _pid;
    dispatch_queue_t _workQueue;
}

- (void)appendLog:(NSString *)line;
- (void)logLine:(NSString *)fmt, ...;
- (void)logNearMiss:(NSString *)reason bucket:(const nm_bucket *)b;
- (void)reRun;
- (void)rescan;
- (void)stopScan;
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
       always sit above the scroll view and stay pinned to the window edges. */

    _rerunButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_rerunButton setTitle:@"RE-RUN" forState:UIControlStateNormal];
    [_rerunButton setTitleColor:[UIColor greenColor] forState:UIControlStateNormal];
    _rerunButton.frame = CGRectMake(10.0, 18.0, 80.0, 30.0);
    _rerunButton.autoresizingMask = UIViewAutoresizingFlexibleRightMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_rerunButton addTarget:self action:@selector(reRun) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_rerunButton];

    _stopButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_stopButton setTitle:@"STOP" forState:UIControlStateNormal];
    [_stopButton setTitleColor:[UIColor redColor] forState:UIControlStateNormal];
    _stopButton.frame = CGRectMake(self.view.bounds.size.width - 90.0, 18.0, 80.0, 30.0);
    _stopButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_stopButton addTarget:self action:@selector(stopScan) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_stopButton];

    _rescanButton = [UIButton buttonWithType:UIButtonTypeSystem];
    [_rescanButton setTitle:@"RESCAN" forState:UIControlStateNormal];
    [_rescanButton setTitleColor:[UIColor yellowColor] forState:UIControlStateNormal];
    _rescanButton.frame = CGRectMake(self.view.bounds.size.width - 180.0, 18.0, 80.0, 30.0);
    _rescanButton.autoresizingMask = UIViewAutoresizingFlexibleLeftMargin | UIViewAutoresizingFlexibleBottomMargin;
    [_rescanButton addTarget:self action:@selector(rescan) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_rescanButton];

    [self.view bringSubviewToFront:_rerunButton];
    [self.view bringSubviewToFront:_rescanButton];
    [self.view bringSubviewToFront:_stopButton];
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

- (void)logLine:(NSString *)fmt, ...
{
    va_list ap;
    va_start(ap, fmt);
    NSString *line = [[NSString alloc] initWithFormat:fmt arguments:ap];
    va_end(ap);
    [self appendLog:line];
}

- (void)logNearMiss:(NSString *)reason bucket:(const nm_bucket *)b
{
    [self logLine:@"rejected: %@ = %llu", reason, b->total];
    for (int i = 0; i < b->count; i++)
        [self logLine:@"  e.g. 0x%llx", b->ex[i]];
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

    if (!ctx.k1779 || !ctx.k7301) {
        [self logLine:@"FAIL: class pointers unavailable"];
        return;
    }

    if (gStopRequested.load()) {
        [self logLine:@"scan stopped"];
        return;
    }

    [self logLine:@"scanning heap..."];

    uint64_t candidates[MAX_CANDIDATES];
    scan_report rep;
    CFAbsoluteTime tScan = CFAbsoluteTimeGetCurrent();
    int count = scan_candidates(task, ctx.k1779, ctx.k7301, candidates, MAX_CANDIDATES, &rep);
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
    [self logLine:@"  k1779 hits      = %llu", rep.k1779_hits];
    [self logLine:@"  predicate passes= %d (cap %d)", count < 0 ? 0 : count, MAX_CANDIDATES];

    if (count < 0) {
        diag_log("diag: scan aborted by stop");
        [self logLine:@"scan stopped"];
        return;
    }

    nm_bucket nmDeref, nmZeroQuat, nmPosRange;
    memset(&nmDeref, 0, sizeof(nmDeref));
    memset(&nmZeroQuat, 0, sizeof(nmZeroQuat));
    memset(&nmPosRange, 0, sizeof(nmPosRange));
    int accepted = 0;

    for (int i = 0; i < count; i++) {
        if (gStopRequested.load()) {
            [self logLine:@"scan stopped"];
            return;
        }

        uint64_t v = candidates[i];

        /* +0xC0 must dereference to a readable quaternion. */
        uint64_t quatPtr = 0;
        if (!mr_read64(task, v + 0xC0, &quatPtr) || quatPtr == 0) {
            nm_add(&nmDeref, v);
            continue;
        }

        mr_quat quat;
        memset(&quat, 0, sizeof(quat));
        if (!mr_read_quat(task, quatPtr + 0x78, &quat)) {
            nm_add(&nmDeref, v);
            continue;
        }

        /* A zero quaternion means the object is destroyed but still on the heap. */
        if (quat.x == 0.0f && quat.y == 0.0f && quat.z == 0.0f && quat.w == 0.0f) {
            nm_add(&nmZeroQuat, v);
            continue;
        }

        uint64_t h0 = rd64(task, v + 0x28);
        uint64_t h1 = h0 ? rd64(task, h0 + 0x18) : 0;
        mr_vec3 posA;
        memset(&posA, 0, sizeof(posA));
        if (!h1 || !mr_read_vec3(task, h1 + 0xD4, &posA) || !diag_vec3_in_range(posA)) {
            nm_add(&nmPosRange, v);
            continue;
        }

        /* ---- accepted: live actor, print the full detail block ---- */
        accepted++;

        uint64_t b = rd64(task, v + 0x30);
        uint64_t bp = rd64(task, b + 0x40);

        mr_quat q = quat;
        float qlen = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        const char *qnorm = (fabsf(qlen - 1.0f) <= 0.01f) ? "PASS" : "FAIL";

        [self logLine:@"candidate[%d] ACCEPT @ 0x%llx", i, v];
        [self logLine:@"  V               = 0x%llx", v];
        [self logLine:@"  read64(V)       = 0x%llx", rd64(task, v)];
        [self logLine:@"  read64(V+0xC0)  = 0x%llx", quatPtr];
        [self logLine:@"  quat_ptr        = 0x%llx", quatPtr];
        [self logLine:@"  quat            = (%.4f, %.4f, %.4f, %.4f) |q|=%.4f %s",
                   q.x, q.y, q.z, q.w, qlen, qnorm];
        [self logLine:@"  pos_a hop0      = read64(V+0x28)    = 0x%llx", h0];
        [self logLine:@"  pos_a hop1      = read64(hop0+0x18) = 0x%llx", h1];
        [self logLine:@"  pos_a xyz       = @hop1+0xD4        = (%.2f, %.2f, %.2f)",
                   posA.x, posA.y, posA.z];
        [self logLine:@"  pos_b hop0      = read64(V+0x30)    = 0x%llx", b];
        [self logLine:@"  pos_b hop1      = read64(hop0+0x40) = 0x%llx", bp];

        mr_vec3 posB;
        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, bp + 0x10, &posB);
        [self logLine:@"  pos_b0 xyz      = @hop1+0x10        = (%.2f, %.2f, %.2f)",
                   posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, bp + 0x1C, &posB);
        [self logLine:@"  pos_b1 xyz      = @hop1+0x1C        = (%.2f, %.2f, %.2f)",
                   posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, bp + 0x2C, &posB);
        [self logLine:@"  pos_b2 xyz      = @hop1+0x2C        = (%.2f, %.2f, %.2f)",
                   posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, bp + 0x38, &posB);
        [self logLine:@"  pos_b3 xyz      = @hop1+0x38        = (%.2f, %.2f, %.2f)",
                   posB.x, posB.y, posB.z];

        [self logLine:@"  self            = %d", (int)rd8(task, v + 0x128)];
        [self logLine:@"  td5878 i0/i1/i2 = %d / %d / %d",
                   (int)rd32(task, b + 0x10), (int)rd32(task, b + 0x14), (int)rd32(task, b + 0x18)];
    }

    [self logLine:@"--- near-misses ---"];
    [self logLine:@"k7301 link reject  = %llu", rep.link_rejected];
    for (int i = 0; i < rep.link_example_count; i++)
        [self logLine:@"  e.g. 0x%llx", rep.link_examples[i]];
    [self logNearMiss:@"+0xC0 deref failed" bucket:&nmDeref];
    [self logNearMiss:@"all-zero quat (dead)" bucket:&nmZeroQuat];
    [self logNearMiss:@"pos out of range" bucket:&nmPosRange];

    [self logLine:@"scan complete: %d accepted / %d predicate passes", accepted, count];
    diag_log("diag: scan complete");
}

@end
