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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_CANDIDATES 256
#define SCAN_CHUNK_SIZE (1u << 20)
#define CLASS_REGION_RADIUS 0x100000ULL

/* --- BFS field hunter ---------------------------------------------------- */
#define HUNT_CAND_LIMIT    8
#define HUNT_OBJ_BUDGET    48
#define HUNT_MAX_DEPTH     2
#define HUNT_L0_SIZE       0x148     /* V+0x00 .. V+0x148 */
#define HUNT_LN_SIZE       0x100     /* p+0x00 .. p+0x100, level 1 and 2 */
#define HUNT_QUAT_MAX      8
#define HUNT_POS_MAX       8
#define HUNT_INT_MAX       3
#define HUNT_HITS_PER_OBJ  (HUNT_QUAT_MAX + HUNT_POS_MAX + HUNT_INT_MAX)
#define HUNT_HITS_PER_CAND (HUNT_OBJ_BUDGET * HUNT_HITS_PER_OBJ)
#define HUNT_PTR_MIN       0x100000000ULL
#define HUNT_PTR_SPAN      0x40000000ULL
#define HUNT_POS_LIMIT     5000.0f
#define HUNT_PATH_LEN      128
#define HUNT_TEXT_LEN      72
#define HUNT_TAIL_LEN      128
#define SCREEN_HIT_LIMIT   80

enum { HUNT_QUAT = 0, HUNT_POS = 1, HUNT_INT100 = 2 };
enum { DIFF_NEW = 0, DIFF_SAME = 1, DIFF_CHANGED = 2 };

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

/* One visited object in the breadth-first walk. level 0 is the candidate
   itself, level 1 a pointer taken from it, level 2 a pointer from one of
   those. path is the full route, e.g. "V+0x18->0x11abd6bf0+0x20->0x11abd7120". */
typedef struct {
    uint64_t addr;
    int      level;
    char     path[HUNT_PATH_LEN];
} hunt_obj;

/* One field hit inside a visited object, with its pre-rendered value text and
   the differential suffix ("", " SAME", " CHANGED (old -> new)"). */
typedef struct {
    int      kind;
    int      obj;
    uint32_t off;
    int      nv;
    float    v[4];
    int      diff;
    char     text[HUNT_TEXT_LEN];
    char     tail[HUNT_TAIL_LEN];
} hunt_hit;

static uint64_t rd64(mach_port_t task, uint64_t addr)
{
    uint64_t v = 0;
    mr_read64(task, addr, &v);
    return v;
}

static uint64_t dist64(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
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

/* A qword is treated as an object pointer when it is 16-byte aligned and sits
   between the first usable heap page and the end of the UnityFramework span.
   That excludes small ints, tags, and metadata addresses in the Mach-O. */
static bool hunt_ptr_ok(uint64_t p, uint64_t uf_base)
{
    if (p < HUNT_PTR_MIN)
        return false;
    if (p > uf_base + HUNT_PTR_SPAN)
        return false;
    if ((p & 0xF) != 0)
        return false;
    return true;
}

/* Reads as much of the object as the mapping allows. Heap objects can end at a
   page boundary, so a failed full-size read is retried with smaller sizes
   instead of discarding the object. */
static size_t hunt_read_obj(mach_port_t task, uint64_t addr, uint8_t *buf, size_t want)
{
    size_t trySize = want;
    for (int guard = 0; guard < 8; guard++) {
        if (trySize < 8)
            trySize = 8;
        if (mr_read(task, addr, buf, trySize))
            return trySize;
        if (trySize == 8)
            break;
        trySize /= 2;
    }
    return 0;
}

/* Breadth-first pointer walk, depth 2, deduplicated across the whole walk and
   capped at maxobjs objects. Returns the number of objects recorded. */
static int hunt_walk(mach_port_t task, uint64_t v, uint64_t uf_base,
                     hunt_obj *objs, int maxobjs)
{
    if (maxobjs < 1)
        return 0;

    int n = 0;
    memset(&objs[0], 0, sizeof(objs[0]));
    objs[0].addr = v;
    objs[0].level = 0;
    snprintf(objs[0].path, sizeof(objs[0].path), "V");
    n = 1;

    int levelBegin = 0;
    int levelEnd = 1;

    for (int depth = 1; depth <= HUNT_MAX_DEPTH && n < maxobjs; depth++) {
        int nextBegin = n;
        uint8_t buf[HUNT_L0_SIZE];

        for (int i = levelBegin; i < levelEnd && n < maxobjs; i++) {
            if (gStopRequested.load())
                return n;

            size_t want = objs[i].level == 0 ? HUNT_L0_SIZE : HUNT_LN_SIZE;
            size_t have = hunt_read_obj(task, objs[i].addr, buf, want);
            if (have == 0)
                continue;

            for (size_t off = 0; off + 8 <= have && n < maxobjs; off += 8) {
                uint64_t p = 0;
                memcpy(&p, buf + off, sizeof(p));
                if (!hunt_ptr_ok(p, uf_base))
                    continue;

                bool dup = false;
                for (int k = 0; k < n; k++) {
                    if (objs[k].addr == p) {
                        dup = true;
                        break;
                    }
                }
                if (dup)
                    continue;

                hunt_obj *o = &objs[n];
                memset(o, 0, sizeof(*o));
                o->addr = p;
                o->level = depth;
                snprintf(o->path, sizeof(o->path), "%s+0x%02zx->0x%llx",
                         objs[i].path, off, (unsigned long long)p);
                n++;
            }
        }

        if (n == nextBegin)
            break;
        levelBegin = nextBegin;
        levelEnd = n;
    }

    return n;
}

/* Scans one object's bytes for the three field signatures. */
static int hunt_object(const uint8_t *buf, size_t size, int objidx,
                       hunt_hit *out, int maxout)
{
    int n = 0;

    /* a) unit quaternion: 16-byte aligned, 4 finite floats, 0.9 <= |q| <= 1.1 */
    int quats = 0;
    for (size_t off = 0; off + 16 <= size && quats < HUNT_QUAT_MAX && n < maxout; off += 16) {
        float f[4];
        memcpy(f, buf + off, sizeof(f));
        if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2]) || !isfinite(f[3]))
            continue;

        float len = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3]);
        if (!(len >= 0.9f && len <= 1.1f))
            continue;

        hunt_hit *h = &out[n++];
        memset(h, 0, sizeof(*h));
        h->kind = HUNT_QUAT;
        h->obj = objidx;
        h->off = (uint32_t)off;
        h->nv = 4;
        memcpy(h->v, f, sizeof(f));
        snprintf(h->text, sizeof(h->text), "(%.4f,%.4f,%.4f,%.4f)", f[0], f[1], f[2], f[3]);
        quats++;
    }

    /* b) position: 4-byte aligned vec3, finite, |comp| <= 5000, not all zero.
       A magnitude floor drops small integers reinterpreted as floats (1e-40
       denormals), which would otherwise fill every object. */
    int poss = 0;
    for (size_t off = 0; off + 12 <= size && poss < HUNT_POS_MAX && n < maxout; off += 4) {
        float f[3];
        memcpy(f, buf + off, sizeof(f));
        if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2]))
            continue;
        if (fabsf(f[0]) > HUNT_POS_LIMIT || fabsf(f[1]) > HUNT_POS_LIMIT ||
            fabsf(f[2]) > HUNT_POS_LIMIT)
            continue;

        float mag = fmaxf(fabsf(f[0]), fmaxf(fabsf(f[1]), fabsf(f[2])));
        if (mag < 0.001f)
            continue;

        hunt_hit *h = &out[n++];
        memset(h, 0, sizeof(*h));
        h->kind = HUNT_POS;
        h->obj = objidx;
        h->off = (uint32_t)off;
        h->nv = 3;
        memcpy(h->v, f, sizeof(f));
        snprintf(h->text, sizeof(h->text), "(%.3f,%.3f,%.3f)", f[0], f[1], f[2]);
        poss++;
    }

    /* c) int32 == 100: health-style counter */
    int ints = 0;
    for (size_t off = 0; off + 4 <= size && ints < HUNT_INT_MAX && n < maxout; off += 4) {
        uint32_t u = 0;
        memcpy(&u, buf + off, sizeof(u));
        if (u != 100u)
            continue;

        hunt_hit *h = &out[n++];
        memset(h, 0, sizeof(*h));
        h->kind = HUNT_INT100;
        h->obj = objidx;
        h->off = (uint32_t)off;
        h->nv = 1;
        h->v[0] = 100.0f;
        snprintf(h->text, sizeof(h->text), "100");
        ints++;
    }

    return n;
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

    /* Previous scan's hit values, keyed by "path|kind|offset". Lives across
       scans so RESCAN can label hits SAME or CHANGED. */
    NSMutableDictionary *_hitCache;
}

- (void)appendLog:(NSString *)line;
- (void)logLine:(NSString *)fmt, ...;
- (void)vlogLine:(NSString *)fmt, ...;
- (NSString *)hitLine:(const hunt_hit *)h objs:(const hunt_obj *)objs;
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
    _hitCache = [NSMutableDictionary dictionary];

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

/* Verbose file only: object walk, hits that did not fit on screen. */
- (void)vlogLine:(NSString *)fmt, ...
{
    va_list ap;
    va_start(ap, fmt);
    NSString *line = [[NSString alloc] initWithFormat:fmt arguments:ap];
    va_end(ap);

    diag_scan_log(line);
}

- (NSString *)hitLine:(const hunt_hit *)h objs:(const hunt_obj *)objs
{
    const char *kind = "QUAT  ";
    if (h->kind == HUNT_POS)
        kind = "POS   ";
    else if (h->kind == HUNT_INT100)
        kind = "INT100";

    if (h->kind == HUNT_INT100)
        return [NSString stringWithFormat:@"%s %s @+0x%02x%s",
                kind, objs[h->obj].path, (unsigned)h->off, h->tail];

    if (h->kind == HUNT_QUAT)
        return [NSString stringWithFormat:@"%s %s @+0x%02x q=%s%s",
                kind, objs[h->obj].path, (unsigned)h->off, h->text, h->tail];

    return [NSString stringWithFormat:@"%s %s @+0x%02x %s%s",
            kind, objs[h->obj].path, (unsigned)h->off, h->text, h->tail];
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

/* Worker queue: all reads, the heap scan and the BFS hunt happen off the main
   thread; every UI write goes through appendLog, which hops to the main queue. */
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

    if (!ctx.uf_base) {
        [self logLine:@"FAIL: UnityFramework Mach-O not found"];
        return;
    }

    /* Bootstrap checks stay on screen, compressed to four lines. */
    [self logLine:@"uf_base = 0x%llx  def_table = 0x%llx", ctx.uf_base, ctx.def_table];
    [self logLine:@"K_1779 = 0x%llx  K_7301 = 0x%llx", ctx.k1779, ctx.k7301];
    [self logLine:@"name = %s (%s) size:%s token:%s  name_ptr:%s", ctx.name,
               ctx.name_ok ? "PASS" : "FAIL",
               ctx.instance_size_ok ? "PASS" : "FAIL",
               ctx.token_ok ? "PASS" : "FAIL",
               ctx.name_ptr_ok ? "PASS" : "FAIL"];
    [self vlogLine:@"metadata_base = 0x%llx", ctx.metadata_base];

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

    if (count == 0) {
        [self logLine:@"hunt complete: 0 objects / 0 quat / 0 pos / 0 int100"];
        return;
    }

    /* The full candidate list is useful in the file only. */
    for (int i = 0; i < count; i++)
        [self vlogLine:@"cand[%d] V=0x%llx cfg=%d", i, candidates[i].v,
                    (int)candidates[i].config_link];

    /* --- candidate selection: config_link first, then distinct V+0x28 --- */
    int sel[HUNT_CAND_LIMIT];
    int nsel = 0;
    uint64_t seen[HUNT_CAND_LIMIT + 1];
    int nseen = 0;

    for (int i = 0; i < count && nsel < HUNT_CAND_LIMIT; i++) {
        if (!candidates[i].config_link)
            continue;
        uint64_t f = rd64(task, candidates[i].v + 0x28);
        sel[nsel++] = i;
        if (nseen < HUNT_CAND_LIMIT + 1)
            seen[nseen++] = f;
    }

    for (int i = 0; i < count && nsel < HUNT_CAND_LIMIT; i++) {
        if (candidates[i].config_link)
            continue;

        uint64_t f = rd64(task, candidates[i].v + 0x28);
        bool dup = false;
        for (int k = 0; k < nseen; k++) {
            if (seen[k] == f) {
                dup = true;
                break;
            }
        }
        if (dup)
            continue;

        if (nseen < HUNT_CAND_LIMIT + 1)
            seen[nseen++] = f;
        sel[nsel++] = i;
    }

    [self vlogLine:@"hunt: %d of %d candidates selected (budget %d objects each)",
                nsel, count, HUNT_OBJ_BUDGET];

    hunt_obj *objs = (hunt_obj *)calloc((size_t)nsel * HUNT_OBJ_BUDGET, sizeof(hunt_obj));
    hunt_hit *hits = (hunt_hit *)calloc((size_t)nsel * HUNT_HITS_PER_CAND, sizeof(hunt_hit));
    int objCount[HUNT_CAND_LIMIT];
    int hitCount[HUNT_CAND_LIMIT];
    memset(objCount, 0, sizeof(objCount));
    memset(hitCount, 0, sizeof(hitCount));

    if (!objs || !hits) {
        free(objs);
        free(hits);
        [self logLine:@"hunt: out of memory"];
        return;
    }

    /* --- BFS walk + signature hunt --------------------------------------- */
    int totalObjs = 0;
    int totalHits = 0;

    for (int s = 0; s < nsel; s++) {
        if (gStopRequested.load()) {
            free(objs);
            free(hits);
            [self logLine:@"scan stopped"];
            return;
        }

        uint64_t v = candidates[sel[s]].v;
        hunt_obj *o = objs + (size_t)s * HUNT_OBJ_BUDGET;
        hunt_hit *h = hits + (size_t)s * HUNT_HITS_PER_CAND;

        [self vlogLine:@"SEL[%d] cand[%d] V=0x%llx cfg=%d", s, sel[s], v,
                    (int)candidates[sel[s]].config_link];

        int nobj = hunt_walk(task, v, ctx.uf_base, o, HUNT_OBJ_BUDGET);
        objCount[s] = nobj;
        totalObjs += nobj;

        uint8_t buf[HUNT_L0_SIZE];
        int hc = 0;

        for (int k = 0; k < nobj; k++) {
            if (gStopRequested.load()) {
                free(objs);
                free(hits);
                [self logLine:@"scan stopped"];
                return;
            }

            size_t want = o[k].level == 0 ? HUNT_L0_SIZE : HUNT_LN_SIZE;
            memset(buf, 0, sizeof(buf));
            size_t have = hunt_read_obj(task, o[k].addr, buf, want);

            [self vlogLine:@"OBJ[%d] lvl=%d %s 0x%llx%s", k, o[k].level, o[k].path,
                        (unsigned long long)o[k].addr,
                        have ? "" : "  <unreadable>"];
            if (have == 0)
                continue;

            int got = hunt_object(buf, have, k, h + hc, HUNT_HITS_PER_CAND - hc);
            hc += got;
        }

        hitCount[s] = hc;
        totalHits += hc;

        [self vlogLine:@"cand[%d] V=0x%llx objs=%d hits=%d", sel[s], v, nobj, hc];
    }

    /* --- differential + screen output ------------------------------------ */
    BOOL capped = totalHits > SCREEN_HIT_LIMIT;
    NSMutableDictionary *prevCache = _hitCache ?: [NSMutableDictionary dictionary];
    NSMutableDictionary *newCache = [NSMutableDictionary dictionaryWithCapacity:(NSUInteger)totalHits];

    int nNew = 0;
    int nSame = 0;
    int nChanged = 0;
    int shown = 0;
    int totalQuat = 0;
    int totalPos = 0;
    int totalInt = 0;

    for (int s = 0; s < nsel; s++) {
        uint64_t v = candidates[sel[s]].v;
        hunt_obj *o = objs + (size_t)s * HUNT_OBJ_BUDGET;
        hunt_hit *h = hits + (size_t)s * HUNT_HITS_PER_CAND;
        int hc = hitCount[s];

        int cq = 0, cp = 0, ci = 0;
        for (int k = 0; k < hc; k++) {
            if (h[k].kind == HUNT_QUAT)
                cq++;
            else if (h[k].kind == HUNT_POS)
                cp++;
            else
                ci++;
        }
        totalQuat += cq;
        totalPos += cp;
        totalInt += ci;

        /* Differential: compare each hit against the previous scan's value. */
        for (int k = 0; k < hc; k++) {
            hunt_hit *p = &h[k];
            NSString *key = [NSString stringWithFormat:@"%s|%d|%u",
                             o[p->obj].path, p->kind, (unsigned)p->off];
            NSString *oldText = prevCache[key];
            NSString *newText = [NSString stringWithUTF8String:p->text];

            if (!oldText) {
                p->diff = DIFF_NEW;
                p->tail[0] = '\0';
                nNew++;
            } else if ([oldText isEqualToString:newText]) {
                p->diff = DIFF_SAME;
                snprintf(p->tail, sizeof(p->tail), " SAME");
                nSame++;
            } else {
                p->diff = DIFF_CHANGED;
                snprintf(p->tail, sizeof(p->tail), " CHANGED (%s -> %s)",
                         [oldText UTF8String], p->text);
                nChanged++;
            }
            newCache[key] = newText;
        }

        int budget = capped ? ((candidates[sel[s]].config_link || shown == 0) ?
                               SCREEN_HIT_LIMIT : 0) : hc;
        int candShown = 0;

        [self logLine:@"cand[%d] V=0x%llx cfg=%d objs=%d hits=%d q:%d p:%d i:%d",
                   sel[s], v, (int)candidates[sel[s]].config_link,
                   objCount[s], hc, cq, cp, ci];

        /* CHANGED first, then SAME, then never-seen: the moving fields are the
           interesting ones when the screen is truncated. */
        static const int order[3] = { DIFF_CHANGED, DIFF_SAME, DIFF_NEW };
        for (int pass = 0; pass < 3; pass++) {
            for (int k = 0; k < hc; k++) {
                if (h[k].diff != order[pass])
                    continue;

                NSString *line = [self hitLine:&h[k] objs:o];
                if (candShown < budget && (!capped || shown < SCREEN_HIT_LIMIT)) {
                    [self logLine:@"%@", line];
                    candShown++;
                    shown++;
                } else {
                    [self vlogLine:@"%@", line];
                }
            }
        }

        if (capped && candShown < hc)
            [self logLine:@"  ... %d more hit(s) for this candidate in diag_scan.log",
                       hc - candShown];
    }

    _hitCache = newCache;

    [self logLine:@"hunt complete: %d objects / %d quat / %d pos / %d int100",
               totalObjs, totalQuat, totalPos, totalInt];
    [self vlogLine:@"diff: %d new / %d same / %d changed (previous keys %lu)",
                nNew, nSame, nChanged, (unsigned long)prevCache.count];

    free(objs);
    free(hits);

    diag_log("diag: scan complete");
}

@end
