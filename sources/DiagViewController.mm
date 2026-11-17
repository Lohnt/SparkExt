#import "DiagViewController.h"
#import "MemRead.h"
#import "Il2Cpp.h"
#import "DiagLog.h"

#import <UIKit/UIKit.h>
#include <atomic>
#include <mach/mach.h>
#include <mach/vm_map.h>
#include <mach/vm_region.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_CANDIDATES 64
#define SCAN_CHUNK_SIZE (1u << 20)

/* Set from the main thread by the STOP button; polled by every loop. */
static std::atomic<bool> gStopRequested(false);

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

/* Returns the number of candidates found, or -1 if a stop was requested. */
static int scan_candidates(mach_port_t task, uint64_t k1779, uint64_t k7301,
                           uint64_t *out, int maxout)
{
    int found = 0;
    vm_address_t addr = 0x100000000ULL;

    uint8_t *chunk = (uint8_t *)malloc(SCAN_CHUNK_SIZE);
    if (!chunk)
        return 0;

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

        if ((info.protection & VM_PROT_READ) && size >= 8 && found < maxout) {
            vm_address_t cursor = addr;
            vm_size_t remaining = size;
            int failures = 0;

            while (remaining >= 8 && found < maxout) {
                if (gStopRequested.load())
                    goto stopped;

                vm_size_t want = remaining < SCAN_CHUNK_SIZE ? remaining : SCAN_CHUNK_SIZE;
                if (mr_read(task, cursor, chunk, want)) {
                    failures = 0;
                    for (vm_size_t off = 0; off + 8 <= want; off += 8) {
                        if (gStopRequested.load())
                            goto stopped;

                        uint64_t value = 0;
                        memcpy(&value, chunk + off, sizeof(value));
                        if (value != k1779)
                            continue;

                        uint64_t candidate = cursor + off;
                        uint64_t link = rd64(task, candidate + 0x18);
                        if (link != 0 && rd64(task, link) == k7301) {
                            out[found++] = candidate;
                            if (found >= maxout)
                                break;
                        }
                    }
                } else {
                    if (++failures >= 3)
                        break;
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
    return found;

stopped:
    free(chunk);
    return -1;
}

@interface DiagViewController () {
    UITextView *_textView;
    UIButton *_rerunButton;
    UIButton *_stopButton;
    BOOL _started;
    mach_port_t _task;
    pid_t _pid;
    dispatch_queue_t _workQueue;
}

- (void)appendLog:(NSString *)line;
- (void)logLine:(NSString *)fmt, ...;
- (void)reRun;
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

- (void)stopScan
{
    gStopRequested.store(true);
}

/* Worker queue: find and attach to Critical Ops, at most 30 attempts. */
- (void)attachLoop
{
    diag_log("diag: attaching");
    [self logLine:@"looking for CriticalOps..."];

    int lastRc = 0;

    for (int attempt = 0; attempt < 30; attempt++) {
        if (gStopRequested.load()) {
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

        sleep(2);
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
    int count = scan_candidates(task, ctx.k1779, ctx.k7301, candidates, MAX_CANDIDATES);

    if (count < 0) {
        [self logLine:@"scan stopped"];
        return;
    }

    for (int i = 0; i < count; i++) {
        if (gStopRequested.load()) {
            [self logLine:@"scan stopped"];
            return;
        }

        uint64_t v = candidates[i];
        [self logLine:@"candidate[%d] @ 0x%llx", i, v];

        uint64_t quatPtr = rd64(task, v + 0xC0);
        mr_quat quat;
        memset(&quat, 0, sizeof(quat));
        mr_read_quat(task, quatPtr + 0x78, &quat);
        [self logLine:@"  quat = (%.3f, %.3f, %.3f, %.3f)", quat.x, quat.y, quat.z, quat.w];

        uint64_t posAPtr = rd64(task, rd64(task, v + 0x28) + 0x18);
        mr_vec3 posA;
        memset(&posA, 0, sizeof(posA));
        mr_read_vec3(task, posAPtr + 0xD4, &posA);
        [self logLine:@"  pos_a = (%.2f, %.2f, %.2f)", posA.x, posA.y, posA.z];

        uint64_t b = rd64(task, v + 0x30);
        uint64_t posBPtr = rd64(task, b + 0x40);

        mr_vec3 posB;
        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, posBPtr + 0x10, &posB);
        [self logLine:@"  pos_b0 = (%.2f, %.2f, %.2f)", posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, posBPtr + 0x1C, &posB);
        [self logLine:@"  pos_b1 = (%.2f, %.2f, %.2f)", posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, posBPtr + 0x2C, &posB);
        [self logLine:@"  pos_b2 = (%.2f, %.2f, %.2f)", posB.x, posB.y, posB.z];

        memset(&posB, 0, sizeof(posB));
        mr_read_vec3(task, posBPtr + 0x38, &posB);
        [self logLine:@"  pos_b3 = (%.2f, %.2f, %.2f)", posB.x, posB.y, posB.z];

        [self logLine:@"  self = %d", (int)rd8(task, v + 0x128)];
        [self logLine:@"  td5878_i0 = %d", (int)rd32(task, b + 0x10)];
        [self logLine:@"  td5878_i1 = %d", (int)rd32(task, b + 0x14)];
        [self logLine:@"  td5878_i2 = %d", (int)rd32(task, b + 0x18)];
    }

    [self logLine:@"scan complete: %d candidates", count];
}

@end
