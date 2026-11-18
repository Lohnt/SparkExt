#import "EspDraw.h"
#import "DiagPrivate.h"
#import "DiagLog.h"
#import "OverlayWindow.h"

#import <UIKit/UIKit.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Round 29 ESP v1 draw path.

   The failure this replaces: the overlay used to read the same mutable cache
   the scan threads wrote, so any scan activity between the humanoid capture and
   the redraw could leave it showing an empty list. Now completion builds one
   immutable snapshot and publishes it with a single atomic store. The overlay
   only ever reads the snapshot pointer; scans only ever write the staging
   buffer in DiagViewController.h. */

typedef struct {
    CFAbsoluteTime stamp;          /* completion time, drives the age text */
    int fovDeg;                    /* horizontal FOV used for this snapshot */
    int camSrc;                    /* 0 none, 1 fresh forward, 2 cached forward */
    int rootN;                     /* humanoid roots copied from staging */
    BOOL haveCam;
    float camP[3];
    float camF[3];
    float roots[ESP_MAX_BOXES * 3];
} esp_snapshot;

/* Immutable once published: never written again and never freed. A scan can
   leak at most one ~1.6 KB snapshot, and that is what makes the swap safe for
   a draw that is still reading the previous one. */
static std::atomic<const esp_snapshot *> gEspSnapshot(NULL);

static void esp_publish(const esp_snapshot *s)
{
    gEspSnapshot.store(s, std::memory_order_release);
}

static const esp_snapshot *esp_current(void)
{
    return gEspSnapshot.load(std::memory_order_acquire);
}

typedef struct {
    CGRect rect;
    CGPoint feet;                  /* projected root: box bottom centre */
    float root[3];
    float dist;
} esp_box;

typedef struct {
    float root[3];
    float zv;
    int reason;                    /* 0 behind, 1 crown, 2 too short */
} esp_skip;

/* Internal to this file; declared here because espCompleteScan and espRedraw
   both forward into the reporting path. */
@interface DiagViewController ()
- (void)espFrameSnapshot:(const esp_snapshot *)s report:(BOOL)report;
- (void)espRedraw;
- (int)espFovDegrees;
- (NSString *)espFovTag;
@end

/* World -> overlay point. right/upv/fwd are the camera basis, sx is
   (W/2)/tan(fovX/2), so a point on the optical axis lands in the middle of the
   screen with square pixels. */
static bool esp_project_point(const float *p, const float *cam,
                              const float *right, const float *upv,
                              const float *fwd, float sx,
                              float cx, float cy, CGPoint *out)
{
    float vx = p[0] - cam[0];
    float vy = p[1] - cam[1];
    float vz = p[2] - cam[2];

    float zv = vx * fwd[0] + vy * fwd[1] + vz * fwd[2];
    if (!isfinite(zv) || zv <= ESP_NEAR_PLANE)
        return false;

    float xv = vx * right[0] + vy * right[1] + vz * right[2];
    float yv = vx * upv[0] + vy * upv[1] + vz * upv[2];

    out->x = cx + (xv / zv) * sx;
    out->y = cy - (yv / zv) * sx;
    return isfinite(out->x) && isfinite(out->y);
}

/* Project every staged root at this screen size. behind counts roots rejected
   by the near-plane test (z_v <= ESP_NEAR_PLANE); skipped counts roots that
   passed it but produced no usable box (crown behind, sub-pixel height). */
static int esp_build_boxes(const esp_snapshot *s, CGSize size,
                           esp_box *boxes, int boxCap,
                           esp_skip *skips, int skipCap,
                           int *behindOut, int *skippedOut)
{
    int drawn = 0;
    int behind = 0;
    int skipped = 0;

    if (behindOut)
        *behindOut = 0;
    if (skippedOut)
        *skippedOut = 0;

    if (!s || !boxes || boxCap <= 0 || s->rootN <= 0 || !s->haveCam)
        return 0;

    float W = (float)size.width;
    float H = (float)size.height;
    if (W < 1.0f || H < 1.0f)
        return 0;

    float tanHalf = tanf((float)s->fovDeg * 0.5f * (float)M_PI / 180.0f);
    if (!(tanHalf > 1e-4f))
        tanHalf = 1e-4f;

    float sx = (W * 0.5f) / tanHalf;
    float cx = W * 0.5f;
    float cy = H * 0.5f;

    const float *cam = s->camP;
    float fwd[3] = { s->camF[0], s->camF[1], s->camF[2] };
    float fl = sqrtf(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
    if (!(fl > 1e-4f))
        return 0;
    fwd[0] /= fl;
    fwd[1] /= fl;
    fwd[2] /= fl;

    /* right = normalize(cross(fwd, (0,1,0))) = normalize(-fwd.z, 0, fwd.x) */
    float right[3] = { -fwd[2], 0.0f, fwd[0] };
    float rl = sqrtf(right[0] * right[0] + right[2] * right[2]);
    if (!(rl > 1e-4f))
        return 0;   /* camera looking straight up or down: no screen basis */

    right[0] /= rl;
    right[2] /= rl;

    /* upv = cross(right, fwd) */
    float upv[3] = { right[1] * fwd[2] - right[2] * fwd[1],
                     right[2] * fwd[0] - right[0] * fwd[2],
                     right[0] * fwd[1] - right[1] * fwd[0] };

    int n = s->rootN < ESP_MAX_BOXES ? s->rootN : ESP_MAX_BOXES;

    for (int i = 0; i < n; i++) {
        const float *root = s->roots + (size_t)i * 3;
        float crown[3] = { root[0], root[1] + ESP_BODY_HEIGHT, root[2] };
        CGPoint bot;
        CGPoint top;

        if (!esp_project_point(root, cam, right, upv, fwd, sx, cx, cy, &bot)) {
            behind++;
            if (skips && skipped + behind <= skipCap) {
                int k = skipped + behind - 1;
                skips[k].root[0] = root[0];
                skips[k].root[1] = root[1];
                skips[k].root[2] = root[2];
                skips[k].zv = (root[0] - cam[0]) * fwd[0] +
                              (root[1] - cam[1]) * fwd[1] +
                              (root[2] - cam[2]) * fwd[2];
                skips[k].reason = 0;
            }
            continue;
        }

        if (!esp_project_point(crown, cam, right, upv, fwd, sx, cx, cy, &top)) {
            skipped++;
            if (skips && skipped + behind <= skipCap) {
                int k = skipped + behind - 1;
                skips[k].root[0] = root[0];
                skips[k].root[1] = root[1];
                skips[k].root[2] = root[2];
                skips[k].zv = 0.0f;
                skips[k].reason = 1;
            }
            continue;
        }

        float h = (float)fabs(top.y - bot.y);
        if (h < ESP_MIN_BOX_PX) {
            skipped++;
            if (skips && skipped + behind <= skipCap) {
                int k = skipped + behind - 1;
                skips[k].root[0] = root[0];
                skips[k].root[1] = root[1];
                skips[k].root[2] = root[2];
                skips[k].zv = 0.0f;
                skips[k].reason = 2;
            }
            continue;
        }

        if (drawn >= boxCap)
            break;

        float w = h * ESP_BOX_WIDTH_RATIO;

        esp_box *b = &boxes[drawn];
        b->rect = CGRectMake(bot.x - w * 0.5f, fminf(bot.y, top.y), w, h);
        b->feet = bot;
        b->root[0] = root[0];
        b->root[1] = root[1];
        b->root[2] = root[2];

        float dx = root[0] - cam[0];
        float dy = root[1] - cam[1];
        float dz = root[2] - cam[2];
        b->dist = sqrtf(dx * dx + dy * dy + dz * dz);
        drawn++;
    }

    if (behindOut)
        *behindOut = behind;
    if (skippedOut)
        *skippedOut = skipped;

    return drawn;
}

@implementation DiagViewController (ESP)

- (int)espFovDegrees
{
    static const int kFovs[ESP_FOV_COUNT] = { 40, 60, 80, 90, 100, 110, 120, 130 };
    int i = _espFovIdx;
    if (i < 0 || i >= ESP_FOV_COUNT)
        i = ESP_FOV_DEFAULT;
    return kFovs[i];
}

- (NSString *)espFovTag
{
    return [NSString stringWithFormat:@"F%d", [self espFovDegrees]];
}

/* ESP RESCAN: the additive button. Shows the overlay and then runs exactly the
   pipeline RESCAN runs. Nothing here touches a published snapshot: a stale one
   stays visible (with a growing age) until this scan completes and replaces it
   in one swap. */
- (void)espRescan
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self espRescan]; });
        return;
    }

    diag_log("diag: esp rescan pressed");

    _espMode = YES;
    _espStageN = 0;
    _espStageHaveCam = NO;
    _espStageCamSrc = 0;

    OverlayWindow *overlay = [OverlayWindow shared];
    [overlay espShow:YES];

    if (!esp_current()) {
        [overlay espClear];
        [overlay espSetRects:NULL distances:NULL count:0
                      status:@"scanning..." fov:[self espFovTag]];
    }

    if (!_espTimer) {
        _espTimer = [NSTimer scheduledTimerWithTimeInterval:1.0
                                                     target:self
                                                   selector:@selector(espTick)
                                                   userInfo:nil
                                                    repeats:YES];
    }

    _textView.text = @"";
    [self logLine:@"ESP RESCAN: pipeline start, fovX=%d", [self espFovDegrees]];
    [self startRun];
}

/* FOV calibration: new immutable snapshot derived from the current one, so the
   draw path still sees exactly one published snapshot. */
- (void)fovChanged:(UISegmentedControl *)sender
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self fovChanged:sender]; });
        return;
    }

    int idx = (int)sender.selectedSegmentIndex;
    if (idx < 0 || idx >= ESP_FOV_COUNT)
        idx = ESP_FOV_DEFAULT;

    _espFovIdx = idx;

    const esp_snapshot *cur = esp_current();
    if (cur) {
        esp_snapshot *next = (esp_snapshot *)malloc(sizeof(*next));
        if (next) {
            memcpy(next, cur, sizeof(*next));
            next->fovDeg = [self espFovDegrees];
            esp_publish(next);
        }
    }

    diag_log("diag: fov changed");

    [self logLine:@"ESP fovX=%d (redraw from cached scan)", [self espFovDegrees]];
    [self espRedraw];
}

- (void)espTick
{
    if (!_espMode) {
        [_espTimer invalidate];
        _espTimer = nil;
        return;
    }
    [self espRedraw];
}

- (void)espRotated:(NSNotification *)note
{
    if (!_espMode)
        return;

    __weak typeof(self) weakSelf = self;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(0.25 * NSEC_PER_SEC)),
                   dispatch_get_main_queue(), ^{
        [weakSelf espRedraw];
    });
}

/* Worker thread, called from the TRS pass: this scan's camera decision, with
   the same fallback the old live cache used - a fresh position plus the
   previous forward still counts as a camera. Roots are staged separately. */
- (void)espStageCamera:(const float *)camP haveP:(bool)haveCamP
                     f:(const float *)camF haveF:(bool)haveCamF
{
    _espStageN = 0;
    _espStageHaveCam = NO;
    _espStageCamSrc = 0;

    const esp_snapshot *prev = esp_current();
    BOOL hadFwd = (prev && prev->haveCam) ? YES : NO;

    if (haveCamP && camP) {
        _espStageCamP[0] = camP[0];
        _espStageCamP[1] = camP[1];
        _espStageCamP[2] = camP[2];

        if (haveCamF && camF) {
            _espStageCamF[0] = camF[0];
            _espStageCamF[1] = camF[1];
            _espStageCamF[2] = camF[2];
            _espStageHaveCam = YES;
            _espStageCamSrc = 1;
        } else if (hadFwd) {
            /* Position refreshed, the previous forward is still the best guess. */
            _espStageCamF[0] = prev->camF[0];
            _espStageCamF[1] = prev->camF[1];
            _espStageCamF[2] = prev->camF[2];
            _espStageHaveCam = YES;
            _espStageCamSrc = 2;
        }
    }
}

/* Worker thread, after the pipeline returned: one snapshot from the staging
   buffer, one atomic swap, then the completion report on the main thread. */
- (void)espCompleteScan
{
    esp_snapshot *s = (esp_snapshot *)calloc(1, sizeof(*s));

    if (!s) {
        /* 1.6 KB; theoretical, but the completion line is unconditional. */
        static esp_snapshot fallback;
        memset(&fallback, 0, sizeof(fallback));
        s = &fallback;
    }

    s->stamp = CFAbsoluteTimeGetCurrent();
    s->fovDeg = [self espFovDegrees];
    s->camSrc = _espStageCamSrc;
    s->haveCam = _espStageHaveCam ? YES : NO;
    s->rootN = _espStageN;

    if (s->rootN < 0)
        s->rootN = 0;
    if (s->rootN > ESP_MAX_BOXES)
        s->rootN = ESP_MAX_BOXES;

    memcpy(s->camP, _espStageCamP, sizeof(s->camP));
    memcpy(s->camF, _espStageCamF, sizeof(s->camF));

    if (s->rootN > 0)
        memcpy(s->roots, _espStageRoots, (size_t)s->rootN * 3 * sizeof(float));

    esp_publish(s);

    const char *src = !s->haveCam ? "none" : (s->camSrc == 2 ? "cache" : "T");

    [self vlogLine:@"esp: cam=(%.3f,%.3f,%.3f) fwd=(%.3f,%.3f,%.3f) src=%s roots=%d",
                s->camP[0], s->camP[1], s->camP[2],
                s->camF[0], s->camF[1], s->camF[2],
                src, s->rootN];

    dispatch_async(dispatch_get_main_queue(), ^{
        [self espFrameSnapshot:s report:YES];
    });
}

/* STOP / EXIT: drop the snapshot and hide the overlay. Scans keep running to
   their next checkpoint; they only ever wrote the staging buffer. */
- (void)espSnapshotClear
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self espSnapshotClear]; });
        return;
    }

    esp_publish(NULL);

    _espMode = NO;
    _espStageN = 0;
    _espStageHaveCam = NO;
    _espStageCamSrc = 0;

    if (_espTimer) {
        [_espTimer invalidate];
        _espTimer = nil;
    }

    OverlayWindow *overlay = [OverlayWindow shared];
    [overlay espClear];
    [overlay espSetRects:NULL distances:NULL count:0 status:nil fov:nil];
    [overlay espShow:NO];
}

- (void)espRedraw
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self espRedraw]; });
        return;
    }

    if (!_espMode)
        return;

    const esp_snapshot *s = esp_current();
    if (!s)
        return;   /* stopped or not completed yet: overlay already cleared */

    [self espFrameSnapshot:s report:NO];
}

/* Main thread: draw one snapshot and, at completion, report it. The report is
   the user's completion flag, so it is printed even when nothing was drawn. */
- (void)espFrameSnapshot:(const esp_snapshot *)s report:(BOOL)report
{
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{
            [self espFrameSnapshot:s report:report];
        });
        return;
    }

    OverlayWindow *overlay = [OverlayWindow shared];
    CGSize size = overlay.bounds.size;
    if (size.width < 1.0 || size.height < 1.0)
        size = [UIScreen mainScreen].bounds.size;

    static float lastW = -1.0f;
    static float lastH = -1.0f;
    if ((float)size.width != lastW || (float)size.height != lastH) {
        lastW = (float)size.width;
        lastH = (float)size.height;

        int fovDeg = s ? s->fovDeg : [self espFovDegrees];
        float tanHalf = tanf((float)fovDeg * 0.5f * (float)M_PI / 180.0f);
        if (!(tanHalf > 1e-4f))
            tanHalf = 1e-4f;

        [self vlogLine:@"esp: overlay %.0fx%.0f fovX=%d scale=%.1f",
                    (double)size.width, (double)size.height,
                    fovDeg, (double)(((float)size.width * 0.5f) / tanHalf)];
    }

    esp_box boxes[ESP_MAX_BOXES];
    esp_skip skips[ESP_MAX_BOXES];
    int behind = 0;
    int skipped = 0;

    memset(boxes, 0, sizeof(boxes));
    memset(skips, 0, sizeof(skips));

    int drawn = esp_build_boxes(s, size, boxes, ESP_MAX_BOXES,
                                skips, ESP_MAX_BOXES, &behind, &skipped);

    CGRect rects[ESP_MAX_BOXES];
    float dists[ESP_MAX_BOXES];
    for (int i = 0; i < drawn; i++) {
        rects[i] = boxes[i].rect;
        dists[i] = boxes[i].dist;
    }

    int roots = s ? s->rootN : 0;
    BOOL haveCam = s ? s->haveCam : NO;
    int fov = s ? s->fovDeg : [self espFovDegrees];
    double age = s ? (CFAbsoluteTimeGetCurrent() - s->stamp) : 0.0;

    NSString *status = nil;
    if (roots <= 0)
        status = @"no skeletons this scan";
    else if (!haveCam)
        status = @"no camera this scan";
    else
        status = [NSString stringWithFormat:@"%.1fs  %d boxes / FOV=%d",
                                            age, drawn, fov];

    [overlay espSetRects:rects distances:dists count:drawn
                  status:status
                     fov:[NSString stringWithFormat:@"F%d", fov]];

    if (!report)
        return;

    if (roots <= 0)
        [self logLine:@"ESP done: roots=0 (no skeletons / no camera)"];
    else
        [self logLine:@"ESP done: roots=%d drawn=%d behind=%d fov=%d",
                   roots, drawn, behind, fov];

    for (int i = 0; i < drawn; i++) {
        esp_box *b = &boxes[i];
        [self vlogLine:@"ESP BOX %d root=(%.3f,%.3f,%.3f) screen=(%.1f,%.1f) w=%.1f h=%.1f",
                    i, b->root[0], b->root[1], b->root[2],
                    (double)b->feet.x, (double)b->feet.y,
                    (double)b->rect.size.width, (double)b->rect.size.height];
    }

    /* Offline check for a zero-box scan: which test rejected which root. */
    int nskip = behind + skipped;
    for (int i = 0; i < nskip && i < 16; i++) {
        esp_skip *k = &skips[i];
        const char *why = k->reason == 0 ? "behind"
                        : (k->reason == 1 ? "crown" : "short");
        [self vlogLine:@"ESP SKIP %d root=(%.3f,%.3f,%.3f) zv=%.2f reason=%s",
                    i, k->root[0], k->root[1], k->root[2],
                    (double)k->zv, why];
    }
}

@end
