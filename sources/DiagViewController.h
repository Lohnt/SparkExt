#import <UIKit/UIKit.h>

#include <mach/mach.h>
#include <stdint.h>
#include <sys/types.h>

/* Round 29: the class and its state live here so the split modules - the scan
   pipeline (DiagScan.mm), the ESP snapshot/draw (EspDraw.mm), the system
   window (OverlayWindow.mm) and the UI glue (DiagViewController.mm) - all see
   the same storage. No behaviour change: this is the same ivar block that used
   to sit in a class extension inside the single .mm. */

/* One hashed region of the planetary snapshot. The hash array is owned by the
   snapshot and reused in place on the following scan (ring of one). */
typedef struct {
    uint64_t base;
    uint64_t size;
    uint16_t *hash;
    uint32_t nblocks;
} planet_region;

/* ESP limits. Shared because the scan staging buffer below and the draw
   snapshot in EspDraw.mm are both sized from them. */
#define ESP_MAX_BOXES       128
#define ESP_FOV_COUNT       8
#define ESP_FOV_DEFAULT     4        /* index of 100 degrees */
#define ESP_BODY_HEIGHT     1.8f     /* feet -> crown, metres */
#define ESP_BOX_WIDTH_RATIO 0.25f    /* box width = height / 4 */
#define ESP_MIN_BOX_PX      2.0f
#define ESP_NEAR_PLANE      0.1f
#define ESP_CAM_FWD_MIN     0.5f
#define ESP_CAM_FWD_MAX     1.5f

@interface DiagViewController : UIViewController {
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

    /* VIEW phase: raw 32-bit slot contents from the previous scan, keyed by the
       absolute slot address, and the V picked as the local player's actor. */
    NSMutableDictionary *_viewSlots;
    uint64_t _localV;

    /* Round 23: per-actor slot cache (V+0x00..0x148), the dedicated cache for
       the T+0xb0 table, and a scan counter for the frozen-table marker. */
    NSMutableDictionary *_actorSlots;
    NSMutableDictionary *_tableSlots;
    int _scanCount;

    /* Round 24: the planetary hash snapshot, kept across scans (ring of one). */
    planet_region *_planetRegs;
    int _planetNRegs;

    /* Round 25: last scan's position per position-address (mover tracking) and
       the insertion order used to keep that history at 200k addresses. */
    NSMutableDictionary *_moverHist;
    NSMutableArray *_moverOrder;

    /* Round 26: per-address SELF distance from the previous scan (PERSIST),
       the posAddr list of this scan's top movers, dumped again next scan
       (MOVE AFTER), and the previous scan's camera P (GHOST rule). */
    NSMutableDictionary *_selfPrev;
    NSMutableArray *_moverWatch;
    float _prevSelfP[3];
    BOOL _prevSelfValid;

    /* Round 27: humanoid array bases seen in the previous scan (STABLE tag). */
    NSMutableSet *_humanPrev;

    /* Round 29 ESP: the overlay toggle and the FOV calibration control. The
       draw itself reads only the immutable snapshot published by EspDraw.mm. */
    UIButton *_espButton;
    UISegmentedControl *_fovControl;
    BOOL _espMode;
    int _espFovIdx;
    NSTimer *_espTimer;

    /* Worker-owned staging buffer: filled by the TRS pass, consumed once by
       espCompleteScan. The overlay never reads these fields, so scan activity
       cannot blank or corrupt what is on screen. */
    float _espStageRoots[ESP_MAX_BOXES * 3];
    int _espStageN;
    int _espStageBind;             /* bind-pose skeletons with no world root */
    BOOL _espStageHaveCam;
    int _espStageCamSrc;           /* 0 none, 1 fresh forward, 2 cached forward */
    float _espStageCamP[3];
    float _espStageCamF[3];
}

@end
