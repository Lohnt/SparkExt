#import "DiagPrivate.h"
#import "MemRead.h"
#import "Il2Cpp.h"
#import "DiagLog.h"
#import "EspDraw.h"

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

/* Round 29: everything that reads the game lives here. The helpers, the heap
   scan, the VIEW phase, the actor census, the PLANETARY differential and the
   TRS hunt are code moved out of DiagViewController.mm; only the ESP hooks
   changed (staging instead of a live cache, completion instead of a capture). */


#define MAX_CANDIDATES 256
#define SCAN_CHUNK_SIZE (1u << 20)
/* Round 47: the candidate walk over the full heap took ~25 s for ~2 GB while
   the TRS pass is already capped. 320 MB is enough to meet every live actor
   on the current build and keeps a scan inside the 2-4 s budget. */
#define SCAN_CAP_BYTES (1024ULL * 1024ULL * 1024ULL)   /* 1 GB: full actor coverage */
#define CLASS_REGION_RADIUS 0x100000ULL

/* --- pointer walk / field hunter ---------------------------------------- */
#define HUNT_OBJ_BUDGET    96        /* objects per candidate */
#define HUNT_MAX_DEPTH     3
#define HUNT_IL2CPP_SIZE   0x180     /* il2cpp object window */
#define HUNT_NATIVE_SIZE   0x300     /* native Unity object window */
#define HUNT_VTABLE_SPAN   0x5400000ULL
#define HUNT_QUAT_MAX      8
#define HUNT_POS_MAX       8
#define HUNT_INT_MAX       3
#define HUNT_HITS_PER_OBJ  (HUNT_QUAT_MAX + HUNT_POS_MAX + HUNT_INT_MAX)
#define HUNT_HITS_PER_CAND (HUNT_OBJ_BUDGET * HUNT_HITS_PER_OBJ)
#define HUNT_PTR_MIN       0x100000000ULL
#define HUNT_PATH_LEN      200
#define HUNT_TEXT_LEN      72
#define HUNT_TAIL_LEN      128
#define HUNT_POS_XZ_LIMIT  3000.0f
#define HUNT_POS_Y_LIMIT   500.0f
#define TOP_CANDIDATES     12
#define SCREEN_TOP_LINES   150

/* --- VIEW phase (local player view state) ------------------------------- */
#define VIEW_OFF_FIELD      0xd0     /* td6362: actor -> view state object */
#define VIEW_CFG_FIELD      0x18     /* actor -> config object */
#define VIEW_SCAN_SIZE      0x300
#define VIEW_WALK_DEPTH     2
#define VIEW_WALK_BUDGET    64       /* objects per walk, root included */
#define VIEW_REGION_MAX     (2 + 2 * VIEW_WALK_BUDGET)
#define VIEW_PATH_LEN       220
#define VIEW_SCREEN_LINES   60
#define VIEW_SPINE_STRIDE   0x58ULL
#define VIEW_SPINE_BYTES    0x58

/* --- round 23: T block, MAT2, actor census, table snapshot -------------- */
#define T_ROWS_FROM         0x0f0
#define T_ROWS_TO           0x180
#define TABLE_OFF_FIELD     0xb0     /* T -> table object (0x12b059000) */
#define TABLE_FALLBACK      0x12b059000ULL
#define TABLE_SCAN_SIZE     0x300
#define TABLE_SCREEN_CHG    20
#define ACTOR_SCAN_SIZE     0x148
#define ACTOR_STRIDE        0x58ULL

/* --- round 24: PLANETARY full-heap differential -------------------------- */
#define PLANET_BLOCK        0x20ULL
#define PLANET_CAP_BYTES    (1024ULL * 1024ULL * 1024ULL)   /* 1 GB: reach bot skeletons */
#define PLANET_REGION_MAX   8192
#define PLANET_CHANGED_MAX  (2u * 1024u * 1024u)
#define PLANET_WIN          0x100    /* dump window per changed block */
#define PLANET_WIN_BACK     0x80
#define PLANET_TEST_BACK    0x20     /* test window starts here, before block */
#define PLANET_TEST_OFF     0x60     /* same bytes inside the dump window */
#define PLANET_TEST_LEN     0x60
#define PLANET_SCREEN_CLU   40
#define PLANET_SCREEN_MOVED 40
#define PLANET_CLUSTER_GAP  0x1000ULL
#define PLANET_HASH_CHUNK   (1u << 20)
#define PLANET_SELF_WIN     0x200
#define PLANET_SELF_RADIUS  3.0f
#define PLANET_MOVER_MAX    200000

/* --- round 26: TRS transform chains, PERSIST, GHOST --------------------- */
#define PLANET_TRS_STRIDE   0x30ULL
#define PLANET_TRS_CHUNK    0x300000u
#define PLANET_TRS_GAP      0x1000ULL
#define PLANET_TRS_ARRAYS   40
#define PLANET_TRS_POS      12
#define PLANET_TRS_OUT_MAX  4000
#define PLANET_TRS_COLLECT  200000   /* slots kept for grouping */
/* Round 46: the TRS walk used to cover the whole selected heap (~1.9 GB,
   ~25 s). ESP mode caps it; regions that hold actor objects and hunt anchors
   are visited first so the humanoid chains are reached inside the cap. */
#define PLANET_TRS_ESP_BYTES (1024ULL * 1024ULL * 1024ULL)   /* 1 GB: reach bot skeletons */
#define PLANET_GHOST_LO     0x16c140000ULL
#define PLANET_GHOST_HI     0x16c150000ULL
#define PLANET_GHOST_MAX    128

/* --- round 27: HUMANOID arrays and projection-matrix hunt ---------------- */
/* Round 30: practice-mode skeletons are ~69 slots long, so the count floor is
   60. Qualification is the head height plus the four bone gaps below. */
#define PLANET_HUMAN_MIN    60      /* slots in a humanoid transform array */
#define PLANET_HUMAN_MAX    40      /* screen lines */
/* Round 49: widened for CriticalOps bind-pose model (head at ~2.7-3.3 from root). */
#define PLANET_HUMAN_HEADLO 0.50f
#define PLANET_HUMAN_HEADHI 3.50f
#define PLANET_HUMAN_OWNER  0x100    /* owner header window before base */
#define PLANET_HUMAN_ROWS   9        /* slot0..slot8 */
#define PLANET_HUMAN_PRE    0x200    /* companion transform window before base */
#define PLANET_HUMAN_PRE_STEP 0x10   /* TRS scan granularity in that window */
#define PLANET_PROJ_SPAN    0x1000
#define PLANET_PROJ_HALF    0x800
#define PLANET_PROJ_EACH    20       /* candidates per anchor */
#define PLANET_PROJ_SCREEN  40
#define PLANET_ANCHOR_MAX   64
#define PLANET_ANCHOR_EPS   0.05f

/* --- il2cpp class metadata ---------------------------------------------- */
#define ACTOR_NAME_EXPECT  "O7D1D7DB39"
#define TD_UNITY_TRANSFORM 13036ULL
#define TD_T5878           5878ULL
#define META_NAME_SPAN     0xE00000ULL
#define NAME_PROBE_END     0x30ULL
#define PARENT_PROBE_END   0x60ULL
#define CHAIN_MAX_HOPS     12

enum { HUNT_QUAT = 0, HUNT_POS = 1, HUNT_INT100 = 2 };
enum { DIFF_NEW = 0, DIFF_SAME = 1, DIFF_CHANGED = 2 };

/* td1779 field table: (instance offset, field type td). Taken from static
   metadata; more reliable than anything read out of the instance. */
typedef struct {
    uint64_t off;
    uint64_t td;
} field_desc;

static const field_desc kActorFields[] = {
    { 0x010, 0ULL },       /* Frida scan: field at +0x10 */
    { 0x014, 0ULL },       /* Frida scan: field at +0x14 */
    { 0x018, 7301ULL },    /* transform / config link */
    { 0x01c, 0ULL },       /* Frida scan: field at +0x1c */
    { 0x020, 7506ULL },    /* transform */
    { 0x024, 0ULL },       /* TEAM BYTE (Frida confirmed) - TD unknown, metadata stripped */
    { 0x026, 0ULL },       /* Frida scan: field at +0x26 */
    { 0x028, 5957ULL },    /* health */
    { 0x02a, 0ULL },       /* Frida scan: field at +0x2a */
    { 0x02c, 0ULL },       /* Frida scan: field at +0x2c */
    { 0x02e, 0ULL },       /* Frida scan: field at +0x2e */
    { 0x030, 5878ULL },    /* viewState (td5878) */
    { 0x034, 0ULL },       /* Frida scan: field at +0x34 */
    { 0x038, 7395ULL },    /* bones / bindPose array */
    { 0x040, 6159ULL },    /* bones / bindPose */
    { 0x042, 0ULL },       /* Frida scan: field at +0x42 */
    { 0x048, 3210ULL },    /* bones / bindPose */
    { 0x050, 3196ULL },    /* bones / bindPose */
    { 0x058, 3421ULL },    /* bones / bindPose */
    { 0x060, 7043ULL },    /* bones / bindPose */
    { 0x068, 3273ULL },
    { 0x070, 9007199337086152ULL },
    { 0x078, 92ULL },
    { 0x080, 1089ULL },
    { 0x088, 2567ULL },
    { 0x090, 6802ULL },
    { 0x098, 5918ULL },
    { 0x0A0, 5700ULL },
    { 0x0A8, 7367ULL },
    { 0x0B0, 6914ULL },
    { 0x0B8, 5856ULL },
    { 0x0C0, 5949ULL },
    { 0x0C8, 6455ULL },
    { 0x0D0, 6362ULL },
    { 0x0D8, 1061ULL },
    { 0x0E0, 2837ULL },
    { 0x0E8, 16ULL },
    { 0x0F0, 6582ULL },
    { 0x0F8, 6866ULL },
    { 0x100, 6784ULL },
    { 0x108, 6024ULL },
    { 0x110, 7013ULL },
    { 0x118, 6562ULL },
    { 0x120, 6427ULL },
    { 0x128, 4028ULL },
    { 0x130, 1309ULL },
    { 0x138, 7679ULL },
    { 0x140, 5837ULL },
};

/* Set from the main thread by the STOP button; polled by every loop. */

/* Pinned with `used` on purpose: clang folds a 5-byte literal handed straight
   to snprintf() into an immediate store, which drops the token from the shipped
   binary. Keeping it as a real object makes the SAME suffix verifiable. */
__attribute__((used)) static const char kTailSame[] = " SAME";

/* Same problem, same fix: the MAT2 row-3 signatures are short literals and one
   of them gets folded away when handed straight to a format argument. */
__attribute__((used)) static const char kRow3Pos[] = "0,0,1,0";
__attribute__((used)) static const char kRow3Neg[] = "0,0,-1,0";

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

/* One visited object in the pointer walk. native == 1 means the first qword
   was a UnityFramework vtable pointer, so the object is a native component;
   those are scanned first and get the larger window. */
typedef struct {
    uint64_t addr;
    uint64_t scanSize;
    int      native;
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

/* ---------------------------------------------------------------- classes */

/* s_TypeInfoDefinitionTable: class(td) = read64(read64(uf_base+0x5086ED8) + td*8) */
static uint64_t tid_class(mach_port_t task, uint64_t tidBase, uint64_t td)
{
    if (!tidBase || td == 0)
        return 0;
    return rd64(task, tidBase + td * 8);
}

static bool meta_range_ok(uint64_t p, uint64_t meta)
{
    return p >= meta && p < meta + META_NAME_SPAN;
}

/* Kept out of line on purpose: clang folds strcmp() against a short literal
   into immediate compares, which hides the tokens from the shipped binary.
   This way "Component" / "Behaviour" / "Transform" stay verifiable strings. */
__attribute__((noinline))
static bool diag_streq(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

/* A metadata string that looks like a class name: short, printable, no spaces. */
static bool meta_name_ok(const char *s)
{
    if (!s)
        return false;

    size_t n = strlen(s);
    if (n == 0 || n > 31)
        return false;

    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x20 || c > 0x7E || c == ' ' || c == '/')
            return false;
    }
    return true;
}

/* Name read at a locked offset: the class's name pointer must land inside the
   metadata blob and point at a plausible identifier. */
static bool class_name_at(mach_port_t task, uint64_t cls, uint64_t nameOff,
                          uint64_t meta, char *out, size_t cap)
{
    if (out && cap)
        out[0] = '\0';
    if (!cls)
        return false;

    uint64_t p = rd64(task, cls + nameOff);
    if (!meta_range_ok(p, meta))
        return false;

    char buf[40];
    memset(buf, 0, sizeof(buf));
    if (!mr_read_cstring(task, p, buf, sizeof(buf)))
        return false;
    if (!meta_name_ok(buf))
        return false;

    if (out && cap)
        snprintf(out, cap, "%s", buf);
    return true;
}

/* Before NAME_OFF is known: probe C+0x00..C+0x30 for any metadata string.
   Returns the name matching `want` when present, else the first plausible one. */
static bool class_name_probe(mach_port_t task, uint64_t cls, uint64_t meta,
                             const char *want, char *out, size_t cap,
                             uint64_t *offOut)
{
    if (out && cap)
        out[0] = '\0';

    char first[40];
    uint64_t firstOff = 0;
    bool haveFirst = false;
    first[0] = '\0';

    for (uint64_t off = 0; off <= NAME_PROBE_END; off += 8) {
        uint64_t p = rd64(task, cls + off);
        if (!meta_range_ok(p, meta))
            continue;

        char buf[40];
        memset(buf, 0, sizeof(buf));
        if (!mr_read_cstring(task, p, buf, sizeof(buf)))
            continue;
        if (!meta_name_ok(buf))
            continue;

        if (want && strcmp(buf, want) == 0) {
            if (out && cap)
                snprintf(out, cap, "%s", buf);
            if (offOut)
                *offOut = off;
            return true;
        }

        if (!haveFirst) {
            snprintf(first, sizeof(first), "%s", buf);
            firstOff = off;
            haveFirst = true;
        }
    }

    if (haveFirst) {
        if (out && cap)
            snprintf(out, cap, "%s", first);
        if (offOut)
            *offOut = firstOff;
        return true;
    }
    return false;
}

/* Round 45: the definition-table slot for a type id is not stable across
   builds/reattaches (13036 read back zero). Verify the slot by class name and,
   when it is empty or wrong, scan the table for a class whose name matches.
   The successful lookup is cached for the process lifetime. */
static uint64_t tid_class_named(mach_port_t task, uint64_t tidBase, uint64_t td,
                                uint64_t meta, uint64_t nameOff,
                                const char *want, uint64_t *foundTd)
{
    static uint64_t cacheBase = 0;
    static uint64_t cacheCls = 0;
    static uint64_t cacheTd = 0;
    static const char *cacheWant = NULL;
    char nm[40];

    if (foundTd)
        *foundTd = td;

    if (!tidBase || !want)
        return 0;

    if (cacheBase == tidBase && cacheCls && cacheWant && diag_streq(cacheWant, want)) {
        if (foundTd)
            *foundTd = cacheTd;
        return cacheCls;
    }

    uint64_t cls = tid_class(task, tidBase, td);

    if (cls >= HUNT_PTR_MIN && (cls & 0x7) == 0) {
        nm[0] = '\0';
        if (class_name_at(task, cls, nameOff, meta, nm, sizeof(nm)) &&
            diag_streq(nm, want)) {
            cacheBase = tidBase;
            cacheCls = cls;
            cacheTd = td;
            cacheWant = want;
            return cls;
        }
    }

    for (uint64_t i = 1; i < 131072ULL; i++) {
        if ((i & 0xFFF) == 0 && gStopRequested.load())
            return 0;

        cls = rd64(task, tidBase + i * 8);
        if (cls < HUNT_PTR_MIN || (cls & 0x7) != 0)
            continue;

        nm[0] = '\0';
        if (!class_name_at(task, cls, nameOff, meta, nm, sizeof(nm)))
            continue;
        if (!diag_streq(nm, want))
            continue;

        cacheBase = tidBase;
        cacheCls = cls;
        cacheTd = i;
        cacheWant = want;
        if (foundTd)
            *foundTd = i;
        return cls;
    }

    return 0;
}

/* Round 45: the definition table is read once by the bootstrap and again by the
   scan. The second read could come back zero, which killed every class lookup
   downstream (Transform null, field chains null, no arrays). A table is only
   accepted when its 1779 slot still points at the actor class the bootstrap
   found. */
static bool tid_base_valid(mach_port_t task, uint64_t base, uint64_t k1779)
{
    if (base < HUNT_PTR_MIN || (base & 0x7) != 0)
        return false;
    if (k1779)
        return rd64(task, base + (uint64_t)IL2CPP_TD_ACTOR * 8ULL) == k1779;
    return rd64(task, base + (uint64_t)IL2CPP_TD_ACTOR * 8ULL) >= HUNT_PTR_MIN;
}

/* Re-read the table global until it agrees with the bootstrap class pointer. */
static uint64_t def_table_verified(mach_port_t task, uint64_t ufBase,
                                   uint64_t k1779, uint64_t seen)
{
    if (tid_base_valid(task, seen, k1779))
        return seen;

    for (int i = 0; i < 8; i++) {
        usleep(250000);
        if (gStopRequested.load())
            break;

        uint64_t b = rd64(task, ufBase + UF_DEF_TABLE_OFF);
        if (tid_base_valid(task, b, k1779))
            return b;
    }
    return 0;
}

/* Builds "Self < Parent < ..." for a class, up to CHAIN_MAX_HOPS, stopping on
   null or unreadable parents. Sets *hasTransform when "Transform" appears. */
static void build_chain(mach_port_t task, uint64_t cls, uint64_t nameOff, uint64_t meta,
                        uint64_t parentOff, char *out, size_t cap, bool *hasTransform)
{
    if (hasTransform)
        *hasTransform = false;
    if (!out || cap == 0)
        return;
    out[0] = '\0';
    if (!cls)
        return;

    size_t used = 0;
    uint64_t cur = cls;

    for (int hop = 0; hop < CHAIN_MAX_HOPS && cur; hop++) {
        char nm[40];
        if (!class_name_at(task, cur, nameOff, meta, nm, sizeof(nm))) {
            if (used == 0)
                snprintf(out, cap, "<?>");
            break;
        }

        if (diag_streq(nm, "Transform") && hasTransform)
            *hasTransform = true;

        size_t add = strlen(nm);
        if (used + add + 4 >= cap)
            break;
        if (used) {
            memcpy(out + used, " < ", 3);
            used += 3;
        }
        memcpy(out + used, nm, add);
        used += add;
        out[used] = '\0';

        if (!parentOff)
            break;

        uint64_t next = rd64(task, cur + parentOff);
        if (!next || next == cur)
            break;
        cur = next;
    }
}

/* ------------------------------------------------------------- BFS hunter */

/* Reads as much of the object as the mapping allows. Heap objects can end at a
   page boundary, so a failed full-size read is retried with smaller sizes
   instead of discarding the object. */
static size_t hunt_read_obj(mach_port_t task, uint64_t addr, uint8_t *buf, size_t want)
{
    size_t trySize = want;
    for (int guard = 0; guard < 10; guard++) {
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

/* Decides whether a pointer target is walkable and how much of it to scan.
   Native Unity objects are recognised by a vtable pointer into UnityFramework
   and get the 0x300 window; il2cpp objects get 0x180; anything else is dropped.
   Returns 0 when the target must not be followed. */
static int hunt_classify(mach_port_t task, uint64_t p, uint64_t uf_base, uint64_t meta,
                         uint64_t *scanSize, int *isNative)
{
    uint64_t p0 = 0;
    if (!mr_read64(task, p, &p0))
        return 0;

    /* vtable -> native Unity component */
    if (p0 >= uf_base && p0 < uf_base + HUNT_VTABLE_SPAN) {
        *scanSize = HUNT_NATIVE_SIZE;
        *isNative = 1;
        return 1;
    }

    /* readable heap pointer -> il2cpp object */
    if (p0 >= HUNT_PTR_MIN && !meta_range_ok(p0, meta)) {
        uint64_t probe = 0;
        if (mr_read64(task, p0, &probe)) {
            *scanSize = HUNT_IL2CPP_SIZE;
            *isNative = 0;
            return 1;
        }
    }

    return 0;
}

/* Breadth-first pointer walk, depth 3, deduplicated across the whole walk and
   capped at maxobjs objects. Never follows V+0x00 (the class pointer) nor any
   target inside the il2cpp metadata blob. Native objects are ordered ahead of
   il2cpp objects at every level so they are traversed first. */
static int hunt_walk(mach_port_t task, uint64_t v, uint64_t uf_base, uint64_t meta,
                     hunt_obj *objs, int maxobjs)
{
    if (maxobjs < 1)
        return 0;

    int n = 0;
    memset(&objs[0], 0, sizeof(objs[0]));
    objs[0].addr = v;
    objs[0].level = 0;
    objs[0].native = 0;
    objs[0].scanSize = HUNT_IL2CPP_SIZE;
    snprintf(objs[0].path, sizeof(objs[0].path), "V");
    n = 1;

    int levelBegin = 0;
    int levelEnd = 1;

    for (int depth = 1; depth <= HUNT_MAX_DEPTH && n < maxobjs; depth++) {
        int nextBegin = n;
        uint8_t buf[HUNT_NATIVE_SIZE];

        for (int i = levelBegin; i < levelEnd && n < maxobjs; i++) {
            if (gStopRequested.load())
                return n;

            size_t have = hunt_read_obj(task, objs[i].addr, buf, (size_t)objs[i].scanSize);
            if (have == 0)
                continue;

            for (size_t off = 0; off + 8 <= have && n < maxobjs; off += 8) {
                /* +0x00 is the Il2CppClass pointer on il2cpp objects and the
                   vtable on native ones; neither is a field, and following it
                   is what read metadata strings as floats. Skip it everywhere;
                   the bytes are still scanned for POS/QUAT/INT100 below. */
                if (off == 0)
                    continue;

                uint64_t p = 0;
                memcpy(&p, buf + off, sizeof(p));

                if (p < HUNT_PTR_MIN)
                    continue;
                if (meta_range_ok(p, meta))
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

                uint64_t scanSize = 0;
                int isNative = 0;
                if (!hunt_classify(task, p, uf_base, meta, &scanSize, &isNative))
                    continue;

                hunt_obj *o = &objs[n];
                memset(o, 0, sizeof(*o));
                o->addr = p;
                o->scanSize = scanSize;
                o->native = isNative;
                o->level = depth;
                snprintf(o->path, sizeof(o->path), "%s+0x%02zx->0x%llx",
                         objs[i].path, off, (unsigned long long)p);
                n++;
            }
        }

        if (n == nextBegin)
            break;

        /* Native objects first inside this level (stable insertion sort). */
        for (int a = nextBegin + 1; a < n; a++) {
            if (!objs[a].native)
                continue;
            hunt_obj tmp = objs[a];
            int b = a;
            while (b > nextBegin && !objs[b - 1].native) {
                objs[b] = objs[b - 1];
                b--;
            }
            objs[b] = tmp;
        }

        levelBegin = nextBegin;
        levelEnd = n;
    }

    return n;
}

/* ------------------------------------------------------------ view state */

/* Slot tags are bit flags so one slot can belong to more than one shape at
   once (a matrix column can hold a position triple, for example). */
enum {
    VIEW_TAG_POS_START  = 1 << 0,
    VIEW_TAG_POS_MEMBER = 1 << 1,
    VIEW_TAG_DIR_START  = 1 << 2,
    VIEW_TAG_DIR4_START = 1 << 3,
    VIEW_TAG_DIR_MEMBER = 1 << 4,
    VIEW_TAG_MAT_START  = 1 << 5,
    VIEW_TAG_MAT_MEMBER = 1 << 6,
    VIEW_TAG_MAT2_POS   = 1 << 7,   /* row 3 == (0,0,1,0) */
    VIEW_TAG_MAT2_NEG   = 1 << 8    /* row 3 == (0,0,-1,0) */
};

/* One chunk of VIEW-phase memory: the view state object (T), the config object
   (C), or a pointer target reached from either. */
typedef struct {
    uint64_t addr;
    uint32_t len;
    char     path[VIEW_PATH_LEN];
} view_region;

typedef struct {
    int slots;
    int pos;
    int dir;
    int mat;
    int mat2;
    int changed;
    int added;
} view_stats;

/* The run #22 position filter, reused verbatim: finite vec3 with |x|,|z| >= 1,
   y either exactly 0 or at least 1, inside the map box, not constant. */
static bool view_pos_ok(const float *f)
{
    if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2]))
        return false;
    if (fabsf(f[0]) < 1.0f || fabsf(f[2]) < 1.0f)
        return false;
    if (fabsf(f[1]) < 1.0f && f[1] != 0.0f)
        return false;
    if (fabsf(f[0]) > HUNT_POS_XZ_LIMIT || fabsf(f[2]) > HUNT_POS_XZ_LIMIT)
        return false;
    if (fabsf(f[1]) > HUNT_POS_Y_LIMIT)
        return false;
    if (f[0] == f[1] && f[1] == f[2])
        return false;
    return true;
}

/* Normalised direction over three floats; the 4th one is often exactly 0.0. */
static bool view_dir_ok(const float *f, float *lenOut)
{
    if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2]))
        return false;

    float len = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (lenOut)
        *lenOut = len;
    return len >= 0.98f && len <= 1.02f;
}

/* 4x4 float block whose elements 12..15 are (0,0,0,1) with a non-zero rotation
   part; plain zero blocks are rejected. */
static bool view_mat_ok(const float *m)
{
    for (int i = 0; i < 16; i++) {
        if (!isfinite(m[i]))
            return false;
    }

    if (!(m[12] == 0.0f && m[13] == 0.0f && m[14] == 0.0f && m[15] == 1.0f))
        return false;

    for (int i = 0; i < 12; i++) {
        if (m[i] != 0.0f)
            return true;
    }
    return false;
}

/* Perspective projection signature: row 3 is (0,0,-1,0) or (0,0,1,0).
   Returns +1/-1 for the sign on the Z row and 0 when it does not match. */
static int view_proj_ok(const float *m)
{
    for (int i = 0; i < 16; i++) {
        if (!isfinite(m[i]))
            return 0;
    }

    if (!(m[12] == 0.0f && m[13] == 0.0f && m[15] == 0.0f))
        return 0;
    if (m[14] != -1.0f && m[14] != 1.0f)
        return 0;

    for (int i = 0; i < 12; i++) {
        if (m[i] != 0.0f)
            return m[14] > 0.0f ? 1 : -1;
    }
    return 0;
}

/* ------------------------------------------------------------ planetary */

/* `planet_region` itself is declared in DiagViewController.h: the planetary
   snapshot now outlives the scan module because the ivars live on the class. */

/* Exactly the planetary position filter: finite, |x| and |z| inside [1,3000],
   |y| <= 500. Softer than view_pos_ok on purpose - a live position may well
   have |y| < 1. */
static bool planet_pos_ok(const float *f)
{
    if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2]))
        return false;

    float ax = fabsf(f[0]);
    float az = fabsf(f[2]);
    if (ax < 1.0f || ax > 3000.0f)
        return false;
    if (az < 1.0f || az > 3000.0f)
        return false;
    if (fabsf(f[1]) > 500.0f)
        return false;
    return true;
}

/* True when v is not within 0.0005 of an integer. Two of the three position
   components have to be fractional for a real entity record. */
static bool planet_frac(float v)
{
    return fabsf(v - roundf(v)) > 0.0005f;
}

/* Tightened position test. The loose planet_pos_ok also matches grid-quantised
   map geometry and (1,1,1) matrix pools, so entity positions additionally need
   an eye-height y and at least two fractional components. */
static bool planet_entity_pos_ok(const float *f)
{
    if (!planet_pos_ok(f))
        return false;

    if (f[1] < 0.3f || f[1] > 30.0f)
        return false;

    int frac = 0;
    for (int i = 0; i < 3; i++) {
        if (planet_frac(f[i]))
            frac++;
    }
    if (frac < 2)
        return false;

    if (f[0] == f[1] && f[1] == f[2])
        return false;

    return true;
}

/* A direction is usable only if it is unit length and not one of the axis
   vectors (|d| >= 0.995 with the other two components <= 0.005). */
static bool planet_entity_dir_ok(const float *f)
{
    if (!view_dir_ok(f, NULL))
        return false;

    for (int i = 0; i < 3; i++) {
        if (fabsf(f[i]) < 0.995f)
            continue;
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        if (fabsf(f[j]) <= 0.005f && fabsf(f[k]) <= 0.005f)
            return false;
    }
    return true;
}

/* Mirrored records differ only in the sign of the direction. Canonicalising
   the sign makes (pos, dir) and (pos, -dir) collide so the first is kept. */
static void planet_canon_dir(const float *d, float *out)
{
    int first = 0;

    for (int i = 0; i < 3; i++) {
        if (fabsf(d[i]) > 0.005f) {
            first = i;
            break;
        }
    }

    float sign = (d[first] < 0.0f) ? -1.0f : 1.0f;
    for (int i = 0; i < 3; i++)
        out[i] = d[i] * sign;
}

/* One moved entity: address of the position triple and the last two values. */
typedef struct {
    uint64_t addr;
    float d;
    float ox, oy, oz;
    float nx, ny, nz;
} mover_rec;

static int mover_cmp_desc(const void *a, const void *b)
{
    const mover_rec *ma = (const mover_rec *)a;
    const mover_rec *mb = (const mover_rec *)b;
    if (ma->d > mb->d)
        return -1;
    if (ma->d < mb->d)
        return 1;
    return 0;
}

/* One Unity transform block: position at +0x00, unit quaternion at +0x10,
   uniform scale triple at +0x20, stride 0x30. */
typedef struct {
    uint64_t addr;
    float px, py, pz;
} trs_slot;

/* A run of consecutive TRS slots, the transform array / chain. */
typedef struct {
    uint64_t base;
    int count;
    int first;
} trs_array;

/* The TRS test: finite in-map position, unit quaternion, uniform scale in
   [0.1,10]. */
static bool planet_trs_ok(const uint8_t *p, float *posOut)
{
    float pos[3];
    float q[4];
    float sc[3];

    memcpy(pos, p, sizeof(pos));
    memcpy(q, p + 0x10, sizeof(q));
    memcpy(sc, p + 0x20, sizeof(sc));

    if (!isfinite(pos[0]) || !isfinite(pos[1]) || !isfinite(pos[2]))
        return false;
    if (fabsf(pos[0]) >= 3000.0f || fabsf(pos[2]) >= 3000.0f)
        return false;
    if (fabsf(pos[1]) >= 50.0f)
        return false;

    for (int i = 0; i < 4; i++) {
        if (!isfinite(q[i]))
            return false;
    }

    float qlen = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qlen < 0.98f || qlen > 1.02f)
        return false;

    for (int i = 0; i < 3; i++) {
        if (!isfinite(sc[i]) || sc[i] < 0.1f || sc[i] > 10.0f)
            return false;
    }

    if (fabsf(sc[0] - sc[1]) > 0.001f || fabsf(sc[1] - sc[2]) > 0.001f ||
        fabsf(sc[0] - sc[2]) > 0.001f)
        return false;

    /* Cleared transform: exact identity at a zero position. These fill freed
       Unity pools by the thousand and would swamp the slot list. */
    if (pos[0] == 0.0f && pos[1] == 0.0f && pos[2] == 0.0f &&
        q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 1.0f &&
        sc[0] == 1.0f && sc[1] == 1.0f && sc[2] == 1.0f)
        return false;

    if (posOut) {
        posOut[0] = pos[0];
        posOut[1] = pos[1];
        posOut[2] = pos[2];
    }
    return true;
}

/* Round 30 humanoid signature. Practice bots ship a 60-70 slot skeleton whose
   first slots are not at the old slot2 head height, but the chain still walks
   feet -> head. Round 31 correction: the four numbers are the ABSOLUTE y of the
   slots that follow the head, not gaps between them. Matched fingerprint:
   head y 0.940, then 0.121, 0.146, 0.138, 0.105. Returns the head slot index,
   or -1 when the array is not a humanoid.

   Round 47: live skeletons have world-position roots (slot0 at map height).
   Check RELATIVE to slot0 (root): head = root_y + 0.92..0.99, then
   root_y + 0.121, 0.146, 0.138, 0.105. Works for both bind-pose and live. */
static int human_sig_start(const uint8_t *raw, int rows)
{
    static const float kRel[4] = { 0.121f, 0.146f, 0.138f, 0.105f };

    if (!raw || rows < PLANET_HUMAN_ROWS)
        return -1;

    float y[PLANET_HUMAN_ROWS];
    for (int k = 0; k < PLANET_HUMAN_ROWS; k++) {
        float v = 0.0f;
        memcpy(&v, raw + (size_t)k * 0x30 + 4, sizeof(v));
        if (!isfinite(v))
            return -1;
        y[k] = v;
    }

    float rootY = y[0];  // slot0 = root (feet)

    for (int i = 0; i <= 3; i++) {
        float headRel = y[i] - rootY;
        if (headRel < PLANET_HUMAN_HEADLO || headRel > PLANET_HUMAN_HEADHI)
            continue;

        int ok = 1;
        for (int k = 0; k < 4; k++) {
            float boneRel = y[i + 1 + k] - rootY;
            if (fabsf(boneRel - kRel[k]) > 0.01f) {
                ok = 0;
                break;
            }
        }

        if (ok)
            return i;
    }

    return -1;
}

/* Round 46: looser humanoid test. The Round 31 fingerprint is exact to 0.01 m
   and some skeleton layouts no longer match it, which left the humanoid list
   empty. This fallback only asks for a plausible head height followed by four
   short bone rows; it is used when the strict signature misses. */
static int human_sig_loose(const uint8_t *raw, int rows)
{
    if (!raw || rows < PLANET_HUMAN_ROWS)
        return -1;

    float y[PLANET_HUMAN_ROWS];
    for (int k = 0; k < PLANET_HUMAN_ROWS; k++) {
        float v = 0.0f;
        memcpy(&v, raw + (size_t)k * 0x30 + 4, sizeof(v));
        if (!isfinite(v))
            return -1;
        y[k] = v;
    }

    for (int i = 0; i <= 3; i++) {
        /* Round 49: widened for CriticalOps bind-pose model (head at ~2.7-3.3). */
        if (y[i] < 0.3f || y[i] > 3.5f)
            continue;

        int ok = 1;
        for (int k = 1; k <= 4; k++) {
            if (y[i + k] < 0.02f || y[i + k] > 0.6f) {
                ok = 0;
                break;
            }
        }

        if (ok)
            return i;
    }

    return -1;
}

/* Round 30: a bind-pose skeleton has its slot0 at exactly (0,0,0) and carries no
   world position. Practice mode keeps the companion world transform in the
   0x200 bytes before the chain, so look there for a TRS-qualified, in-map slot.
   The closest one to the chain wins. */
static bool human_bind_pre(mach_port_t task, uint64_t base, float *out)
{
    if (!out || base <= PLANET_HUMAN_PRE)
        return false;

    uint8_t win[PLANET_HUMAN_PRE];
    memset(win, 0, sizeof(win));

    uint64_t start = base - PLANET_HUMAN_PRE;
    size_t got = hunt_read_obj(task, start, win, PLANET_HUMAN_PRE);
    if (got < 0x30)
        return false;

    for (int off = (int)(got - 0x30); off >= 0; off -= PLANET_HUMAN_PRE_STEP) {
        float pos[3];
        if (!planet_trs_ok(win + off, pos))
            continue;
        if (!planet_pos_ok(pos))
            continue;

        out[0] = pos[0];
        out[1] = pos[1];
        out[2] = pos[2];
        return true;
    }

    return false;
}

/* ------------------------------------------------------- round 34 camera */

/* UnityEngine.Camera lookup. Tuple layout inside the class object: position at
   the slot, unit quaternion 12 or 16 bytes later, uniform scale 16 after that.
   Both spacings are tried because native Transform packs Vector3f without the
   16-byte alignment the bone arrays use. */
static bool cam_trs_probe(const uint8_t *buf, size_t got, float *pos, float *quat)
{
    if (!buf || got < 0x40)
        return false;

    for (size_t b = 0; b + 0x30 <= got; b += 4) {
        float p[3];
        memcpy(p, buf + b, sizeof(p));
        if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]))
            continue;
        if (fabsf(p[0]) < 1.0f || fabsf(p[0]) > 3000.0f)
            continue;
        if (fabsf(p[2]) < 1.0f || fabsf(p[2]) > 3000.0f)
            continue;
        if (p[1] < -2.0f || p[1] > 50.0f)
            continue;

        for (int qo = 12; qo <= 16; qo += 4) {
            size_t qb = b + (size_t)qo;
            if (qb + 16 + 12 > got)
                continue;

            float q[4];
            memcpy(q, buf + qb, sizeof(q));
            if (!isfinite(q[0]) || !isfinite(q[1]) ||
                !isfinite(q[2]) || !isfinite(q[3]))
                continue;

            float ql = sqrtf(q[0] * q[0] + q[1] * q[1] +
                             q[2] * q[2] + q[3] * q[3]);
            if (ql < 0.98f || ql > 1.02f)
                continue;

            float s[3];
            memcpy(s, buf + qb + 16, sizeof(s));
            if (!isfinite(s[0]) || !isfinite(s[1]) || !isfinite(s[2]))
                continue;
            if (s[0] < 0.2f || s[0] > 5.0f ||
                s[1] < 0.2f || s[1] > 5.0f ||
                s[2] < 0.2f || s[2] > 5.0f)
                continue;
            if (fabsf(s[0] - s[1]) > 0.01f || fabsf(s[1] - s[2]) > 0.01f ||
                fabsf(s[0] - s[2]) > 0.01f)
                continue;

            memcpy(pos, p, sizeof(p));
            memcpy(quat, q, sizeof(q));
            return true;
        }
    }

    return false;
}

/* Round 37: world root for a bind-pose skeleton. The chain rows are model
   space, so the owning object is probed instead: a pointer target is read and
   tested for a TRS row with an in-map position, both directly and through its
   managed m_CachedPtr (0x10). */
static bool human_world_pos_ok(const float *p)
{
    if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]))
        return false;
    if (fabsf(p[0]) > 3000.0f || fabsf(p[2]) > 3000.0f)
        return false;
    if (fabsf(p[1]) > 500.0f)
        return false;
    if (p[0] == 0.0f && p[1] == 0.0f && p[2] == 0.0f)
        return false;
    return true;
}

static bool human_probe_ptr(mach_port_t task, uint64_t c, float *out,
                            uint64_t *srcOut)
{
    /* Round 41: a managed Component reaches its native Transform through
       m_CachedPtr, but the field offset varies with the class layout (0x8 for
       some builds, 0x10 for others). Probe the object itself and every
       pointer-sized field in the first 0x48 bytes. */
    static const size_t kOff[8] = { 0x08, 0x10, 0x18, 0x20, 0x28, 0x30,
                                    0x38, 0x40 };
    uint8_t buf[0x400];
    float p[3];

    if (!out || c < HUNT_PTR_MIN)
        return false;

    for (int pass = 0; pass <= 8; pass++) {
        uint64_t addr = c;

        if (pass > 0) {
            addr = rd64(task, c + kOff[pass - 1]);
            if (addr < HUNT_PTR_MIN)
                continue;
        }

        memset(buf, 0, sizeof(buf));
        size_t got = hunt_read_obj(task, addr, buf, sizeof(buf));
        if (got < 0x30)
            continue;

        /* The camera probe insists on |x|,|z| >= 1; a bot near a world axis
           would fail it. Only the finiteness/scale checks matter here. */
        for (size_t b = 0; b + 0x30 <= got; b += 0x10) {
            if (!planet_trs_ok(buf + b, p))
                continue;
            if (!human_world_pos_ok(p))
                continue;

            memcpy(out, p, sizeof(p));
            if (srcOut)
                *srcOut = addr + (uint64_t)b;
            return true;
        }
    }

    return false;
}

/* Pointers in the 0x100-byte owner header before the chain, plus one level of
   their children: the actor root Transform is normally the first component. */
typedef struct {
    uint64_t hdr;
    uint64_t p[4];
    int ncand;
    int tested;
} human_owner_dbg;

static bool human_owner_root(mach_port_t task, uint64_t base, float *out,
                             uint64_t *srcOut, human_owner_dbg *dbg)
{
    if (dbg)
        memset(dbg, 0, sizeof(*dbg));

    if (!out || base <= PLANET_HUMAN_OWNER)
        return false;

    uint64_t hb = base - PLANET_HUMAN_OWNER;
    uint8_t hw[PLANET_HUMAN_OWNER];
    memset(hw, 0, sizeof(hw));
    size_t hh = hunt_read_obj(task, hb, hw, sizeof(hw));
    if (hh < 8)
        return false;

    if (dbg) {
        dbg->hdr = hb;
        for (int i = 0; i < 4 && (size_t)(i + 1) * 8 <= hh; i++)
            memcpy(&dbg->p[i], hw + (size_t)i * 8, sizeof(uint64_t));
    }

    /* Round 41: the skeleton array's header points at the owning GameObject,
       but the world Transform is usually two hops away (GameObject ->
       components -> Transform/m_CachedPtr). Walk that pointer graph breadth
       first, testing every reachable block for a world-space TRS row. */
    uint64_t queue[160];
    int qn = 0;
    int qh = 0;

    for (size_t i = 0; i + 8 <= hh && qn < 160; i += 8) {
        uint64_t v = 0;
        memcpy(&v, hw + i, sizeof(v));
        if (v >= HUNT_PTR_MIN && v <= 0x0000ffffffffffffULL)
            queue[qn++] = v;
    }

    while (qh < qn && qh < 160) {
        uint64_t node = queue[qh++];
        uint8_t blk[0x100];
        memset(blk, 0, sizeof(blk));
        size_t got = hunt_read_obj(task, node, blk, sizeof(blk));

        if (got < 8)
            continue;

        if (dbg)
            dbg->tested = qh;

        /* A Transform block can sit at any 16-byte offset in this object. */
        for (size_t b = 0; b + 0x30 <= got; b += 0x10) {
            float pos[3];
            float xz;

            if (!planet_trs_ok(blk + b, pos))
                continue;
            if (!human_world_pos_ok(pos))
                continue;
            if (fabsf(pos[1]) > 5.0f)
                continue;   /* skybox / flying transform, not a ground actor */

            xz = sqrtf(pos[0] * pos[0] + pos[2] * pos[2]);
            if (xz < 0.5f)
                continue;

            memcpy(out, pos, sizeof(pos));
            if (srcOut)
                *srcOut = node + (uint64_t)b;
            if (dbg)
                dbg->ncand = qn;
            return true;
        }

        for (size_t k = 0; k + 8 <= got && qn < 160; k += 8) {
            uint64_t v = 0;
            memcpy(&v, blk + k, sizeof(v));
            if (v < HUNT_PTR_MIN || v > 0x0000ffffffffffffULL)
                continue;

            bool seen = false;

            for (int i = 0; i < qn; i++) {
                if (queue[i] == v) {
                    seen = true;
                    break;
                }
            }

            if (!seen)
                queue[qn++] = v;
        }
    }

    if (dbg)
        dbg->ncand = qn;

    return false;
}

/* Round 42: a bind-skeleton root must be an actor on the map, not a skybox,
   camera or model-space row. Ground actors sit at |y| <= 5; the 24 m skybox
   transform is the failure this rejects. */
static bool human_root_pos_ok(const float *p, const float *camP)
{
    if (!human_world_pos_ok(p))
        return false;

    if (fabsf(p[1]) > 5.0f)
        return false;

    float xz = sqrtf(p[0] * p[0] + p[2] * p[2]);
    if (xz < 0.5f)
        return false;

    if (camP) {
        float dx = p[0] - camP[0];
        float dy = p[1] - camP[1];
        float dz = p[2] - camP[2];
        if (sqrtf(dx * dx + dy * dy + dz * dz) < 1.0f)
            return false;
    }

    return true;
}

/* Wider companion scan: a filtered TRS row in the 64 KB before the chain, then
   in the 64 KB after it (past the skeleton rows). Nearest address first. */
static bool human_near_trs(mach_port_t task, uint64_t base, int rows,
                           const float *camP, const uint64_t *used, int usedN,
                           float *out, uint64_t *srcOut, uint64_t *distOut)
{
    const uint64_t win = 0x10000;
    uint8_t *buf = (uint8_t *)malloc(win);

    if (!buf || !out)
        return false;

    bool hit = false;

    if (base > win) {
        size_t got = hunt_read_obj(task, base - win, buf, win);

        for (int off = (int)(got - 0x30); off >= 0; off -= 0x10) {
            float pos[3];

            if (!planet_trs_ok(buf + off, pos))
                continue;
            if (!human_root_pos_ok(pos, camP))
                continue;

            uint64_t a = base - win + (uint64_t)off;
            bool skipped = false;

            for (int u = 0; u < usedN; u++) {
                if (used && used[u] == a) {
                    skipped = true;
                    break;
                }
            }

            if (skipped)
                continue;

            memcpy(out, pos, sizeof(pos));
            if (srcOut)
                *srcOut = a;
            if (distOut)
                *distOut = base - a;
            hit = true;
            break;
        }
    }

    if (!hit) {
        uint64_t start = base + (uint64_t)(rows > 0 ? rows : 0) * 0x30ULL;
        size_t got = hunt_read_obj(task, start, buf, win);

        for (int off = 0; off + 0x30 <= (int)got; off += 0x10) {
            float pos[3];

            if (!planet_trs_ok(buf + off, pos))
                continue;
            if (!human_root_pos_ok(pos, camP))
                continue;

            uint64_t a = start + (uint64_t)off;
            bool skipped = false;

            for (int u = 0; u < usedN; u++) {
                if (used && used[u] == a) {
                    skipped = true;
                    break;
                }
            }

            if (skipped)
                continue;

            memcpy(out, pos, sizeof(pos));
            if (srcOut)
                *srcOut = a;
            if (distOut)
                *distOut = a - base;
            hit = true;
            break;
        }
    }

    free(buf);
    return hit;
}

/* Round 42: last resort pairing. Same filters as the near scan, but over every
   TRS slot the pass collected within 64 KB. The 16 MB window let a skybox
   transform 0x44a00 away win; distance and height filters now reject it.
   Near-origin rows, the camera row and roots already claimed by another
   skeleton are rejected. */
static bool human_nearest_slot(const trs_slot *slots, int nslots, uint64_t base,
                               int rows, const float *camP,
                               const uint64_t *used, int usedN,
                               float *out, uint64_t *srcOut,
                               uint64_t *distOut)
{
    const uint64_t maxDist = 0x10000ULL;   /* 64 KB: farther matches are props */

    if (!slots || nslots <= 0 || !out)
        return false;

    uint64_t end = base + (uint64_t)(rows > 0 ? rows : 0) * 0x30ULL;
    uint64_t best = 0;
    uint64_t bestd = ~0ULL;
    const trs_slot *bestSlot = NULL;

    for (int i = 0; i < nslots; i++) {
        uint64_t a = slots[i].addr;

        if (a >= base && a < end)
            continue;   /* the skeleton itself */

        uint64_t d = (a > base) ? (a - base) : (base - a);
        if (d > maxDist || d >= bestd)
            continue;
        if (!human_root_pos_ok(&slots[i].px, camP))
            continue;

        bool skipped = false;

        for (int u = 0; u < usedN; u++) {
            if (used && used[u] == a) {
                skipped = true;
                break;
            }
        }

        if (skipped)
            continue;

        best = a;
        bestd = d;
        bestSlot = &slots[i];
    }

    if (!bestSlot)
        return false;

    out[0] = bestSlot->px;
    out[1] = bestSlot->py;
    out[2] = bestSlot->pz;
    if (srcOut)
        *srcOut = best;
    if (distOut)
        *distOut = bestd;
    return true;
}

/* Rotate (0,0,1) by a unit quaternion: Unity's forward axis. */
static bool cam_fwd_from_quat(const float *q, float *out)
{
    float x = q[0], y = q[1], z = q[2], w = q[3];

    out[0] = 2.0f * (x * z + w * y);
    out[1] = 2.0f * (y * z - w * x);
    out[2] = 1.0f - 2.0f * (x * x + y * y);

    float len = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    if (!isfinite(len) || len < 0.9f || len > 1.1f)
        return false;

    out[0] /= len;
    out[1] /= len;
    out[2] /= len;
    return true;
}

static uint64_t cam_class_find(mach_port_t task, uint64_t tidBase,
                               uint64_t meta, uint64_t nameOff)
{
    uint64_t fallback = 0;

    for (uint64_t td = 1; td < 65536ULL; td++) {
        uint64_t cls = rd64(task, tidBase + td * 8);
        if (!cls)
            continue;

        char nm[32];
        if (!class_name_at(task, cls, nameOff, meta, nm, sizeof(nm)))
            continue;
        if (!diag_streq(nm, "Camera"))
            continue;

        /* Prefer UnityEngine.Camera over game classes named Camera. */
        uint64_t np = rd64(task, cls + nameOff + 8);
        if (meta_range_ok(np, meta)) {
            char ns[32];
            memset(ns, 0, sizeof(ns));
            if (mr_read_cstring(task, np, ns, sizeof(ns)) &&
                diag_streq(ns, "UnityEngine"))
                return cls;
        }

        if (!fallback)
            fallback = cls;
    }

    return fallback;
}

/* Camera instance -> m_CachedPtr/native Camera -> transform. The transform may
   also be a managed Transform wrapper, whose own m_CachedPtr is tested. */
static bool cam_find_transform(mach_port_t task, uint64_t inst, uint64_t tfCls,
                               float *pos, float *quat, uint64_t *xfOut)
{
    static const uint64_t kNativeOff[4] = { 0x10, 0x8, 0x18, 0x20 };
    uint64_t cand[48];
    int nc = 0;

    cand[nc++] = inst;

    /* Managed Camera -> m_CachedPtr (0x10) and neighbours. */
    for (int i = 0; i < 4 && nc < 8; i++) {
        uint64_t v = rd64(task, inst + kNativeOff[i]);
        if (v >= HUNT_PTR_MIN)
            cand[nc++] = v;
    }

    /* Round 36: expand every level-1 pointer's children, not just the first
       four fields, so a native Camera -> GameObject -> Transform chain is
       reachable. Transform wrappers are still tested below. */
    int lvl1 = nc;

    for (int i = 1; i < lvl1 && nc < 48; i++) {
        uint8_t blk[0x100];
        memset(blk, 0, sizeof(blk));
        size_t got = hunt_read_obj(task, cand[i], blk, sizeof(blk));
        for (size_t k = 0; k + 8 <= got && nc < 48; k += 8) {
            uint64_t v = 0;
            memcpy(&v, blk + k, sizeof(v));
            if (v >= HUNT_PTR_MIN)
                cand[nc++] = v;
        }
    }

    for (int i = 0; i < nc; i++) {
        uint64_t c = cand[i];

        uint8_t buf[0x400];
        memset(buf, 0, sizeof(buf));
        size_t got = hunt_read_obj(task, c, buf, sizeof(buf));
        if (got >= 0x40 && cam_trs_probe(buf, got, pos, quat)) {
            if (xfOut)
                *xfOut = c;
            return true;
        }

        if (tfCls && rd64(task, c) == tfCls) {
            uint64_t nat = rd64(task, c + 0x10);
            if (nat >= HUNT_PTR_MIN) {
                memset(buf, 0, sizeof(buf));
                got = hunt_read_obj(task, nat, buf, sizeof(buf));
                if (got >= 0x40 && cam_trs_probe(buf, got, pos, quat)) {
                    if (xfOut)
                        *xfOut = nat;
                    return true;
                }
            }
        }
    }

    return false;
}

static int trs_cmp_addr(const void *a, const void *b)
{
    const trs_slot *x = (const trs_slot *)a;
    const trs_slot *y = (const trs_slot *)b;
    return x->addr < y->addr ? -1 : (x->addr > y->addr ? 1 : 0);
}

static int trs_cmp_count(const void *a, const void *b)
{
    const trs_array *x = (const trs_array *)a;
    const trs_array *y = (const trs_array *)b;

    if (x->count != y->count)
        return x->count > y->count ? -1 : 1;
    return x->base < y->base ? -1 : (x->base > y->base ? 1 : 0);
}

/* Unity projection matrix, row-major indices: m33 ~ 0, a -1 handedness term
   at either m32 (row-major copies, index 14) or m23 (Unity column-major,
   index 11), and both focal terms sane. */
static bool planet_proj_ok(const float *m)
{
    for (int i = 0; i < 16; i++) {
        if (!isfinite(m[i]))
            return false;
    }

    if (fabsf(m[15]) >= 0.02f)
        return false;

    bool one14 = (m[14] >= 0.7f && m[14] <= 1.3f) ||
                 (m[14] <= -0.7f && m[14] >= -1.3f);
    bool one11 = (m[11] >= 0.7f && m[11] <= 1.3f) ||
                 (m[11] <= -0.7f && m[11] >= -1.3f);

    if (!one14 && !one11)
        return false;

    if (m[0] < 0.3f || m[0] > 5.0f)
        return false;
    if (m[5] < 0.3f || m[5] > 5.0f)
        return false;

    return true;
}

/* 16-bit fingerprint of one 0x20 block: XOR-fold its eight u32 words. */
static uint16_t planet_hash_block(const uint8_t *p)
{
    uint32_t h = 0;
    for (int i = 0; i < 8; i++) {
        uint32_t w = 0;
        memcpy(&w, p + (size_t)i * 4, sizeof(w));
        h ^= w;
    }
    return (uint16_t)(h ^ (h >> 16));
}

static int planet_cmp_size(const void *a, const void *b)
{
    const planet_region *ra = (const planet_region *)a;
    const planet_region *rb = (const planet_region *)b;

    if (ra->size < rb->size)
        return -1;
    if (ra->size > rb->size)
        return 1;
    return ra->base < rb->base ? -1 : (ra->base > rb->base ? 1 : 0);
}

static int planet_cmp_base(const void *a, const void *b)
{
    const planet_region *ra = (const planet_region *)a;
    const planet_region *rb = (const planet_region *)b;
    return ra->base < rb->base ? -1 : (ra->base > rb->base ? 1 : 0);
}

static int planet_cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a;
    uint64_t y = *(const uint64_t *)b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

/* Regions are kept in base order, so lookup is a binary search. */
static planet_region *planet_find(planet_region *regs, int n, uint64_t base)
{
    int lo = 0;
    int hi = n - 1;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (regs[mid].base == base)
            return &regs[mid];
        if (regs[mid].base < base)
            lo = mid + 1;
        else
            hi = mid - 1;
    }
    return NULL;
}

/* Enumerates readable+writable regions of at least one block. Returns the
   number collected, or -1 when STOP was pressed. */
static int planet_collect(mach_port_t task, planet_region *out, int maxn)
{
    vm_address_t addr = 0;
    int n = 0;

    for (int guard = 0; guard < 1000000 && n < maxn; guard++) {
        if (gStopRequested.load())
            return -1;

        vm_size_t size = 0;
        vm_region_basic_info_data_64_t info;
        mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
        mach_port_t objectName = MACH_PORT_NULL;

        memset(&info, 0, sizeof(info));
        kern_return_t kr = vm_region_64(task, &addr, &size, VM_REGION_BASIC_INFO_64,
                                        (vm_region_info_t)&info, &count, &objectName);
        if (objectName != MACH_PORT_NULL)
            mach_port_deallocate(mach_task_self(), objectName);
        if (kr != KERN_SUCCESS)
            break;

        if ((info.protection & VM_PROT_READ) && (info.protection & VM_PROT_WRITE) &&
            size >= PLANET_BLOCK) {
            out[n].base = addr;
            out[n].size = size;
            out[n].hash = NULL;
            out[n].nblocks = 0;
            n++;
        }

        vm_address_t next = addr + size;
        if (next <= addr)
            break;
        addr = next;
    }

    return n;
}

static const char *view_tag_name(uint8_t tag)
{
    if (tag & VIEW_TAG_POS_START)
        return "POS";
    if (tag & VIEW_TAG_DIR4_START)
        return "DIR4";
    if (tag & VIEW_TAG_DIR_START)
        return "DIR";
    if (tag & VIEW_TAG_MAT_START)
        return "MAT";
    if (tag & VIEW_TAG_MAT2_POS)
        return "MAT2+";
    if (tag & VIEW_TAG_MAT2_NEG)
        return "MAT2-";
    if (tag & VIEW_TAG_POS_MEMBER)
        return "POS*";
    if (tag & VIEW_TAG_DIR_MEMBER)
        return "DIR*";
    if (tag & VIEW_TAG_MAT_MEMBER)
        return "MAT*";
    return "raw";
}

/* Depth-2 pointer walk for the VIEW phase. Appends the root and everything its
   pointers reach (at most VIEW_WALK_BUDGET objects including the root) to regs,
   skipping +0x00 and any target inside the il2cpp metadata blob. Native Unity
   objects and il2cpp objects are both accepted and both scanned over 0x300. */
static int view_walk(mach_port_t task, uint64_t uf_base, uint64_t meta,
                     view_region *regs, int start, int maxn,
                     uint64_t root, const char *rootPath)
{
    if (!root || start >= maxn)
        return start;

    for (int k = 0; k < start; k++) {
        if (regs[k].addr == root)
            return start;
    }

    regs[start].addr = root;
    regs[start].len = VIEW_SCAN_SIZE;
    snprintf(regs[start].path, sizeof(regs[start].path), "%s", rootPath);

    int n = start + 1;
    int levelBegin = start;
    int levelEnd = n;

    for (int depth = 1; depth <= VIEW_WALK_DEPTH && n < maxn; depth++) {
        int nextBegin = n;

        for (int i = levelBegin; i < levelEnd && n < maxn; i++) {
            if (gStopRequested.load())
                return n;

            uint8_t buf[VIEW_SCAN_SIZE];
            size_t have = hunt_read_obj(task, regs[i].addr, buf, VIEW_SCAN_SIZE);
            if (have == 0)
                continue;

            for (size_t off = 8; off + 8 <= have && n < maxn; off += 8) {
                uint64_t p = 0;
                memcpy(&p, buf + off, sizeof(p));

                if (p < HUNT_PTR_MIN)
                    continue;
                if (meta_range_ok(p, meta))
                    continue;

                bool dup = false;
                for (int k = 0; k < n; k++) {
                    if (regs[k].addr == p) {
                        dup = true;
                        break;
                    }
                }
                if (dup)
                    continue;

                uint64_t scanSize = 0;
                int isNative = 0;
                if (!hunt_classify(task, p, uf_base, meta, &scanSize, &isNative))
                    continue;

                regs[n].addr = p;
                regs[n].len = VIEW_SCAN_SIZE;
                snprintf(regs[n].path, sizeof(regs[n].path), "%s+0x%02zx->0x%llx",
                         regs[i].path, off, (unsigned long long)p);
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

    /* b) world position: 4-byte aligned vec3. x and z must be at least 1 in
       magnitude, y may be exactly 0; the box is |x|,|z| <= 3000, |y| <= 500;
       constant triples are rejected. A window that merely slides one float off
       the previously reported one is suppressed. */
    int poss = 0;
    bool havePrev = false;
    uint32_t prevOff = 0;
    float prevY = 0.0f;
    float prevZ = 0.0f;

    for (size_t off = 0; off + 12 <= size && poss < HUNT_POS_MAX && n < maxout; off += 4) {
        float f[3];
        memcpy(f, buf + off, sizeof(f));

        if (!view_pos_ok(f))
            continue;

        /* Sliding-window suppression: the same triple shifted by 4 bytes. */
        if (havePrev && (uint32_t)off == prevOff + 4 &&
            prevY == f[0] && prevZ == f[1])
            continue;

        hunt_hit *h = &out[n++];
        memset(h, 0, sizeof(*h));
        h->kind = HUNT_POS;
        h->obj = objidx;
        h->off = (uint32_t)off;
        h->nv = 3;
        memcpy(h->v, f, sizeof(f));
        snprintf(h->text, sizeof(h->text), "(%.3f,%.3f,%.3f)", f[0], f[1], f[2]);

        havePrev = true;
        prevOff = (uint32_t)off;
        prevY = f[1];
        prevZ = f[2];
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

        if (rep->bytes_scanned >= SCAN_CAP_BYTES)
            break;

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
            bool capped = false;

            while (remaining >= 8 && found < maxout && !capped) {
                if (gStopRequested.load())
                    goto stopped;

                if (rep->bytes_scanned >= SCAN_CAP_BYTES) {
                    capped = true;
                    break;
                }

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


@interface DiagViewController (CameraMain)
- (BOOL)resolveMainCameraPos:(float *)posOut fwd:(float *)fwdOut kind:(int *)kindOut
                    regions:(const planet_region *)regs count:(int)count;
@end

@implementation DiagViewController (Scan)

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

/* Fills diff status + pre-rendered suffix for every hit, and folds the values
   into the next cache when one is supplied. stats = quat, pos, int, new, same,
   changed. Comparison is always against the previous scan only, so calling
   this twice for the same candidate yields the same labels. */
- (void)diffHits:(hunt_hit *)hits count:(int)hc objs:(const hunt_obj *)objs
            prev:(NSDictionary *)prev next:(NSMutableDictionary *)next
           stats:(int *)stats
{
    for (int k = 0; k < hc; k++) {
        hunt_hit *p = &hits[k];

        if (p->kind == HUNT_QUAT)
            stats[0]++;
        else if (p->kind == HUNT_POS)
            stats[1]++;
        else
            stats[2]++;

        NSString *key = [NSString stringWithFormat:@"%s|%d|%u",
                         objs[p->obj].path, p->kind, (unsigned)p->off];
        NSString *oldText = prev[key];
        NSString *newText = [NSString stringWithUTF8String:p->text];

        if (!oldText) {
            p->diff = DIFF_NEW;
            p->tail[0] = '\0';
            stats[3]++;
        } else if ([oldText isEqualToString:newText]) {
            p->diff = DIFF_SAME;
            memcpy(p->tail, kTailSame, sizeof(kTailSame));
            stats[4]++;
        } else {
            p->diff = DIFF_CHANGED;
            snprintf(p->tail, sizeof(p->tail), " CHANGED (%s -> %s)",
                     [oldText UTF8String], p->text);
            stats[5]++;
        }

        if (next)
            next[key] = newText;
    }
}

/* Main thread: cancel any in-flight work and queue a fresh run on the worker. */
/* Worker queue: all reads, the heap scan and the hunts happen off the main
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

    /* Round 29: ESP staging is worker-owned and reset once per scan. The
       published draw snapshot is never touched from here. */
    if (_espMode) {
        _espStageN = 0;
        _espStageBind = 0;
        _espStageHaveCam = NO;
        _espStageCamSrc = 0;
        _moverRootN = 0;   /* Round 47: mover ESP roots */
    }

    [self vlogLine:@"=== scan start ==="];
    CFAbsoluteTime tRunStart = CFAbsoluteTimeGetCurrent();

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

    /* Round 45: verify the class table before the heap pass, which matches
       every candidate object against its slots. */
    {
        uint64_t tb = def_table_verified(task, ctx.uf_base, ctx.k1779,
                                         ctx.def_table);

        if (!tb) {
            [self logLine:@"FAIL: class table invalid (def_table=0x%llx)",
                        ctx.def_table];
            return;
        }

        if (tb != ctx.def_table) {
            [self vlogLine:@"tidBase recovered = 0x%llx", tb];
            ctx.def_table = tb;
        }
    }

    if (gStopRequested.load()) {
        [self logLine:@"scan stopped"];
        return;
    }

    [self logLine:@"scanning heap..."];
    _scanCount++;

    cand_t candidates[MAX_CANDIDATES];
    scan_report rep;
    CFAbsoluteTime tScan = CFAbsoluteTimeGetCurrent();
    int count = scan_candidates(task, ctx.k1779, ctx.k7301, ctx.def_table,
                                candidates, MAX_CANDIDATES, &rep);
    double scanWall = CFAbsoluteTimeGetCurrent() - tScan;
    diag_log("diag: scan returned");

    [self logLine:@"scan: scanned=%.1f MB cap=%.0f MB elapsed=%.2f",
               (double)rep.bytes_scanned / (1024.0 * 1024.0),
               (double)SCAN_CAP_BYTES / (1024.0 * 1024.0), scanWall];
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

    /* Round 45: the table was verified before the heap pass; reuse it. */
    uint64_t tidBase = ctx.def_table;
    uint64_t meta = ctx.metadata_base;

    /* ---------------- class discovery (file only) --------------------- */
    [self vlogLine:@"--- class discovery ---"];
    [self vlogLine:@"tidBase = 0x%llx  metadata = 0x%llx", tidBase, meta];

    uint64_t nameOff = 0;
    char actorName[40];
    bool haveName = class_name_probe(task, ctx.k1779, meta, ACTOR_NAME_EXPECT,
                                     actorName, sizeof(actorName), &nameOff);
    if (!haveName)
        nameOff = 0x10;   /* Il2CppClass::name */

    /* Round 34: keep the metadata handles for the Camera::main resolver, which
       runs after the heap pass. */
    _ufBase = ctx.uf_base;
    _tidBase = tidBase;
    _metaBase = meta;
    _nameOff = nameOff;

    [self vlogLine:@"class(1779) = 0x%llx", ctx.k1779];
    for (uint64_t off = 0; off <= NAME_PROBE_END; off += 8) {
        uint64_t p = rd64(task, ctx.k1779 + off);
        if (!meta_range_ok(p, meta))
            continue;
        char buf[40];
        memset(buf, 0, sizeof(buf));
        if (!mr_read_cstring(task, p, buf, sizeof(buf)) || !meta_name_ok(buf))
            continue;
        [self vlogLine:@"  probe +0x%02llx 0x%llx \"%s\"%s", off, p, buf,
                    (off == nameOff) ? "  <- NAME_OFF" : ""];
    }

    /* Round 45: resolve Transform by name when its table slot reads back zero.
       The slot index is verified against the class name before use. */
    uint64_t tfTd = TD_UNITY_TRANSFORM;
    uint64_t tfCls = haveName
        ? tid_class_named(task, tidBase, TD_UNITY_TRANSFORM, meta, nameOff,
                          "Transform", &tfTd)
        : tid_class(task, tidBase, TD_UNITY_TRANSFORM);
    char tfName[40];
    tfName[0] = '\0';
    if (tfCls)
        class_name_at(task, tfCls, nameOff, meta, tfName, sizeof(tfName));
    [self vlogLine:@"class(13036) = 0x%llx  name = %s td=%llu", tfCls,
                tfName[0] ? tfName : "?", (unsigned long long)tfTd];

    uint64_t parentOff = 0;
    for (uint64_t off = 0; off <= PARENT_PROBE_END; off += 8) {
        uint64_t p = rd64(task, tfCls + off);
        /* Class objects are at least 8-aligned; 16 is the common case but not
           guaranteed, so 8 is used here to avoid missing the parent. */
        if (p < HUNT_PTR_MIN || (p & 0x7) != 0)
            continue;

        char pnm[40];
        pnm[0] = '\0';
        bool ok = class_name_at(task, p, nameOff, meta, pnm, sizeof(pnm));
        if (!ok)
            ok = class_name_probe(task, p, meta, NULL, pnm, sizeof(pnm), NULL);
        if (!ok)
            continue;

        bool isParent = (diag_streq(pnm, "Component") || diag_streq(pnm, "Behaviour"));
        [self vlogLine:@"  parent? +0x%02llx -> 0x%llx  %s%s", off, p, pnm,
                    isParent ? "  <-- PARENT_OFF" : ""];
        if (isParent && !parentOff)
            parentOff = off;
    }

    char parentTxt[24];
    if (parentOff)
        snprintf(parentTxt, sizeof(parentTxt), "+0x%02llx", (unsigned long long)parentOff);
    else
        snprintf(parentTxt, sizeof(parentTxt), "<none>");

    [self vlogLine:@"discovery %s name=%s NAME_OFF=+0x%02llx PARENT_OFF=%s",
               (haveName && parentOff) ? "PASS" : "FAIL",
               haveName ? actorName : "?", nameOff, parentTxt];

    /* ---------------- field chains (file only) ------------------------ */
    [self vlogLine:@"--- field chains (td1779) ---"];

    uint64_t tfFieldOff = 0;
    uint64_t tfFieldTd = 0;
    int nf = (int)(sizeof(kActorFields) / sizeof(kActorFields[0]));

    for (int i = 0; i < nf; i++) {
        uint64_t off = kActorFields[i].off;
        uint64_t td = kActorFields[i].td;

        if (td > 0x10000000ULL) {
            [self vlogLine:@"field +0x%02llx td=%llu chain: <td out of range>",
                        off, (unsigned long long)td];
            continue;
        }

        uint64_t cls = tid_class(task, tidBase, td);

        /* Round 45: a just-initialised type can still read back zero for a
           moment after attach; give the slot one retry before calling it null.
           When Transform resolved at another index, the same index shift is
           applied to the field's type id. */
        if (!cls) {
            usleep(200000);
            cls = tid_class(task, tidBase, td);
        }
        if (!cls && tfTd != TD_UNITY_TRANSFORM) {
            uint64_t alt = td + tfTd - TD_UNITY_TRANSFORM;
            cls = tid_class(task, tidBase, alt);
            if (cls) {
                [self vlogLine:@"field +0x%02llx td=%llu shifted -> td=%llu",
                            off, (unsigned long long)td,
                            (unsigned long long)alt];
                td = alt;
            }
        }
        if (!cls) {
            [self vlogLine:@"field +0x%02llx td=%llu chain: <class null>",
                        off, (unsigned long long)td];
            continue;
        }

        char chain[192];
        bool hasTf = false;
        build_chain(task, cls, nameOff, meta, parentOff, chain, sizeof(chain), &hasTf);

        if (hasTf && !tfFieldOff) {
            tfFieldOff = off;
            tfFieldTd = td;
        }

        [self vlogLine:@"field +0x%02llx td=%llu chain: %s%s", off,
                    (unsigned long long)td, chain[0] ? chain : "<?>",
                    hasTf ? "  <<< TRANSFORM" : ""];
    }

    if (tfFieldOff)
        [self vlogLine:@"transform field = +0x%02llx td=%llu", tfFieldOff,
                   (unsigned long long)tfFieldTd];
    else
        [self vlogLine:@"transform field = <none>"];

    if (count == 0) {
        [self vlogLine:@"hunt complete: 0 objects / 0 quat / 0 pos / 0 int100"];
        return;
    }

    if (gStopRequested.load()) {
        [self logLine:@"scan stopped"];
        return;
    }

    /* ---------------- hunt every candidate ---------------------------- */
    hunt_obj *objs = (hunt_obj *)calloc(HUNT_OBJ_BUDGET, sizeof(hunt_obj));
    hunt_hit *hits = (hunt_hit *)calloc(HUNT_HITS_PER_CAND, sizeof(hunt_hit));
    uint8_t *buf = (uint8_t *)malloc(HUNT_NATIVE_SIZE);

    if (!objs || !hits || !buf) {
        free(objs);
        free(hits);
        free(buf);
        [self logLine:@"hunt: out of memory"];
        return;
    }

    int candQuat[MAX_CANDIDATES];
    int candPos[MAX_CANDIDATES];
    int candInt[MAX_CANDIDATES];
    int candChg[MAX_CANDIDATES];
    int candObj[MAX_CANDIDATES];
    int candTd[MAX_CANDIDATES][3];
    memset(candQuat, 0, sizeof(candQuat));
    memset(candPos, 0, sizeof(candPos));
    memset(candInt, 0, sizeof(candInt));
    memset(candChg, 0, sizeof(candChg));
    memset(candObj, 0, sizeof(candObj));
    memset(candTd, 0, sizeof(candTd));

    NSMutableDictionary *prevCache = _hitCache ?: [NSMutableDictionary dictionary];
    NSMutableDictionary *newCache = [NSMutableDictionary dictionaryWithCapacity:1024];

    int totalObjs = 0;
    int totalQuat = 0;
    int totalPos = 0;
    int totalInt = 0;
    int totalChg = 0;
    bool stopped = false;

    for (int i = 0; i < count; i++) {
        if (gStopRequested.load()) {
            stopped = true;
            break;
        }

        uint64_t v = candidates[i].v;
        int nobj = hunt_walk(task, v, ctx.uf_base, meta, objs, HUNT_OBJ_BUDGET);
        candObj[i] = nobj;
        totalObjs += nobj;

        int hc = 0;
        for (int k = 0; k < nobj; k++) {
            memset(buf, 0, HUNT_NATIVE_SIZE);
            size_t have = hunt_read_obj(task, objs[k].addr, buf, (size_t)objs[k].scanSize);

            [self vlogLine:@"OBJ[%d.%d] lvl=%d n=%d sz=0x%llx %s 0x%llx%s",
                        i, k, objs[k].level, objs[k].native, objs[k].scanSize,
                        objs[k].path, (unsigned long long)objs[k].addr,
                        have ? "" : "  <unreadable>"];
            if (have == 0)
                continue;

            hc += hunt_object(buf, have, k, hits + hc, HUNT_HITS_PER_CAND - hc);
        }

        int stats[6] = { 0, 0, 0, 0, 0, 0 };
        [self diffHits:hits count:hc objs:objs prev:prevCache next:newCache stats:stats];

        /* td5878 object (field +0x30): reported as i32 at +0x10/+0x14/+0x18. */
        uint32_t u0 = 0, u1 = 0, u2 = 0;
        uint64_t t5878 = rd64(task, v + 0x30);
        if (t5878) {
            mr_read32(task, t5878 + 0x10, &u0);
            mr_read32(task, t5878 + 0x14, &u1);
            mr_read32(task, t5878 + 0x18, &u2);
        }
        candTd[i][0] = (int)(int32_t)u0;
        candTd[i][1] = (int)(int32_t)u1;
        candTd[i][2] = (int)(int32_t)u2;

        candQuat[i] = stats[0];
        candPos[i] = stats[1];
        candInt[i] = stats[2];
        candChg[i] = stats[5];
        totalQuat += stats[0];
        totalPos += stats[1];
        totalInt += stats[2];
        totalChg += stats[5];

        [self vlogLine:@"cand[%d] V=0x%llx cfg=%d objs=%d hits=%d q=%d p=%d i=%d chg=%d td5878=(%d,%d,%d)",
                    i, v, (int)candidates[i].config_link, nobj, hc, stats[0], stats[1],
                    stats[2], stats[5], candTd[i][0], candTd[i][1], candTd[i][2]];

        for (int k = 0; k < hc; k++)
            [self vlogLine:@"%@", [self hitLine:&hits[k] objs:objs]];

        if (candidates[i].config_link) {
            uint64_t cfgObj = rd64(task, v + 0x18);
            uint32_t mx = 0;
            if (cfgObj)
                mr_read32(task, cfgObj + 0x390, &mx);
            [self vlogLine:@"CFG cand[%d] obj=0x%llx +0x390 i32=%d u32=%u", i, cfgObj,
                        (int)(int32_t)mx, mx];
        }
    }

    if (stopped) {
        free(objs);
        free(hits);
        free(buf);
        [self logLine:@"scan stopped"];
        return;
    }

    /* ---------------- compact file summary: one line per candidate ----- */
    for (int i = 0; i < count; i++) {
        [self vlogLine:@"cand[%d] V=0x%llx pos?=%d quat?=%d chg=%d td5878=(%d,%d,%d)",
                   i, candidates[i].v, candPos[i], candQuat[i], candChg[i],
                   candTd[i][0], candTd[i][1], candTd[i][2]];
    }

    [self vlogLine:@"--- TOP by chg ---"];

    int top[TOP_CANDIDATES];
    int topN = 0;
    int rankChg[MAX_CANDIDATES];
    memcpy(rankChg, candChg, sizeof(rankChg));

    for (int slot = 0; slot < TOP_CANDIDATES; slot++) {
        int best = -1;
        for (int i = 0; i < count; i++) {
            if (rankChg[i] < 0)
                continue;
            if (candPos[i] + candQuat[i] == 0)
                continue;
            if (best < 0 || rankChg[i] > rankChg[best])
                best = i;
        }
        if (best < 0)
            break;
        top[topN++] = best;
        rankChg[best] = -1;
    }

    for (int s = 0; s < topN; s++) {
        int i = top[s];
        uint64_t v = candidates[i].v;

        if (gStopRequested.load()) {
            [self logLine:@"scan stopped"];
            break;
        }

        int nobj = hunt_walk(task, v, ctx.uf_base, meta, objs, HUNT_OBJ_BUDGET);
        int hc = 0;

        for (int k = 0; k < nobj; k++) {
            memset(buf, 0, HUNT_NATIVE_SIZE);
            size_t have = hunt_read_obj(task, objs[k].addr, buf, (size_t)objs[k].scanSize);
            if (have == 0)
                continue;
            hc += hunt_object(buf, have, k, hits + hc, HUNT_HITS_PER_CAND - hc);
        }

        int stats[6] = { 0, 0, 0, 0, 0, 0 };
        [self diffHits:hits count:hc objs:objs prev:prevCache next:nil stats:stats];

        [self vlogLine:@"cand[%d] V=0x%llx objs=%d chg=%d", i, v, nobj, stats[5]];

        static const int order[3] = { DIFF_CHANGED, DIFF_SAME, DIFF_NEW };
        for (int pass = 0; pass < 3; pass++) {
            for (int k = 0; k < hc; k++) {
                if (hits[k].diff != order[pass])
                    continue;
                if (hits[k].kind == HUNT_INT100)
                    continue;
                [self vlogLine:@"%@", [self hitLine:&hits[k] objs:objs]];
            }
        }
    }

    _hitCache = newCache;

    [self vlogLine:@"hunt complete: %d objects / %d quat / %d pos / %d int100",
                totalObjs, totalQuat, totalPos, totalInt];
    [self vlogLine:@"diff: %d changed of %d hits across %d candidates",
                totalChg, totalQuat + totalPos + totalInt, count];

    if (!gStopRequested.load()) {
        [self runViewPhaseTask:task ctx:&ctx candidates:candidates count:count
                       candChg:candChg];
    }

    if (!gStopRequested.load())
        [self runActorCensusTask:task candidates:candidates count:count];

    if (!gStopRequested.load())
        [self runPlanetaryTask:task];

    free(objs);
    free(hits);
    free(buf);

    diag_log("diag: scan complete");
    [self logLine:@"scan done in %.2f s", CFAbsoluteTimeGetCurrent() - tRunStart];
}

/* ---------------- VIEW phase ------------------------------------------ */

/* Focused look at the local player's view state. Run #22 established that the
   live position/direction hang off T = read64(V+0xd0) (field td6362), not off
   the actor itself, and that only the local player's graph moves. So T and the
   config object C are scanned together with everything their pointers reach,
   and every 4-byte float slot is diffed against the previous scan. */
- (void)runViewPhaseTask:(mach_port_t)task
                     ctx:(const il2cpp_ctx *)ctx
              candidates:(const cand_t *)cands
                   count:(int)count
                 candChg:(const int *)candChg
{
    [self vlogLine:@"--- VIEW phase ---"];

    /* Item 5: identify the first node of the +0x58-stride actor run. */
    [self viewDumpSpineTask:task meta:ctx->metadata_base candidates:cands
                      count:count];

    /* LOCAL = the config-linked candidate that changed most in the hunt, with
       the previous pick winning ties so RESCAN stays on the same actor. */
    int localIdx = -1;

    for (int i = 0; i < count; i++) {
        if (!cands[i].config_link)
            continue;
        if (localIdx < 0) {
            localIdx = i;
        } else if (candChg[i] > candChg[localIdx]) {
            localIdx = i;
        } else if (candChg[i] == candChg[localIdx] && _localV &&
                   cands[i].v == _localV) {
            localIdx = i;
        }
    }

    view_stats vs;
    memset(&vs, 0, sizeof(vs));

    if (localIdx < 0) {
        [self vlogLine:@"view: no config-linked candidate"];
        [self vlogLine:@"view: %d slots / %d pos / %d dir / %d mat (%d changed)",
                   vs.slots, vs.pos, vs.dir, vs.mat, vs.changed];
        return;
    }

    uint64_t vLocal = cands[localIdx].v;
    uint64_t tObj = rd64(task, vLocal + VIEW_OFF_FIELD);
    uint64_t cfgObj = rd64(task, vLocal + VIEW_CFG_FIELD);
    _localV = vLocal;

    [self vlogLine:@"view LOCAL cand[%d] V=0x%llx chg=%d T=0x%llx C=0x%llx",
                localIdx, (unsigned long long)vLocal, candChg[localIdx],
                (unsigned long long)tObj, (unsigned long long)cfgObj];

    /* Item 1: the T block (screen band, file dump, every unit vec3). */
    [self viewTBlockTask:task t:tObj];

    /* Item 4: the table hanging off T+0xb0 (0x12b059000 in run #22). */
    uint64_t tableObj = tObj ? rd64(task, tObj + TABLE_OFF_FIELD) : 0;
    const char *tableSrc = "T+0xb0";
    if (!tableObj) {
        tableObj = TABLE_FALLBACK;
        tableSrc = "literal";
    }
    uint64_t tableSkip = (tableObj && tableObj != tObj && tableObj != cfgObj)
                         ? tableObj : 0;

    view_region *regs = (view_region *)calloc(VIEW_REGION_MAX, sizeof(view_region));
    if (!regs) {
        [self logLine:@"view: out of memory"];
        return;
    }

    int nregs = 0;
    int tRegs = 0;

    if (tObj) {
        nregs = view_walk(task, ctx->uf_base, ctx->metadata_base, regs, 0,
                          VIEW_REGION_MAX, tObj, "T");
        tRegs = nregs;
    } else {
        [self vlogLine:@"view: T = 0 (V+0x%02x unreadable)", VIEW_OFF_FIELD];
    }

    if (cfgObj) {
        nregs = view_walk(task, ctx->uf_base, ctx->metadata_base, regs, nregs,
                          VIEW_REGION_MAX, cfgObj, "C");
    }

    [self vlogLine:@"view regions = %d (T %d, C %d)", nregs, tRegs, nregs - tRegs];

    NSDictionary *prev = _viewSlots ?: [NSDictionary dictionary];
    NSMutableDictionary *next = [NSMutableDictionary dictionaryWithCapacity:32768];

    int left = VIEW_SCREEN_LINES;
    bool stopped = false;

    for (int i = 0; i < nregs; i++) {
        if (gStopRequested.load()) {
            stopped = true;
            break;
        }
        [self viewScanRegion:&regs[i] task:task prev:prev next:next
                        probe:tObj stats:&vs screenLeft:&left skip:tableSkip];
    }

    NSDictionary *tprev = _tableSlots ?: [NSDictionary dictionary];
    NSMutableDictionary *tnext = [NSMutableDictionary dictionaryWithCapacity:1024];
    [self viewTableTask:task base:tableObj meta:ctx->metadata_base
                   prev:tprev next:tnext src:tableSrc];
    if (!stopped)
        _tableSlots = tnext;

    /* On an aborted pass keep the old snapshot; a partial one would make every
       untouched slot look NEW next time. */
    if (!stopped)
        _viewSlots = next;

    if (left <= 0)
        [self vlogLine:@"  ... view block truncated; full hits in diag_scan.log"];

    [self vlogLine:@"view: %d slots / %d pos / %d dir / %d mat (%d changed)",
                vs.slots, vs.pos, vs.dir, vs.mat, vs.changed];
    [self vlogLine:@"view mat2 = %d / new slots = %d", vs.mat2, vs.added];

    if (stopped)
        [self logLine:@"scan stopped"];

    free(regs);
}

/* Item 5: find the first node of the candidate run spaced 0x58 apart and dump
   its first 0x58 bytes as hex + float rows, file only. */
- (void)viewDumpSpineTask:(mach_port_t)task
                     meta:(uint64_t)meta
               candidates:(const cand_t *)cands
                    count:(int)count
{
    uint64_t head = 0;

    for (int i = 0; i < count && !head; i++) {
        uint64_t v = cands[i].v;
        bool hasFwd = false;
        bool hasBack = false;

        for (int j = 0; j < count; j++) {
            if (j == i)
                continue;
            if (cands[j].v == v + VIEW_SPINE_STRIDE)
                hasFwd = true;
            else if (cands[j].v == v - VIEW_SPINE_STRIDE)
                hasBack = true;
        }

        if (hasFwd && !hasBack)
            head = v;
    }

    if (!head) {
        [self vlogLine:@"VIEW SPINE: no candidate has a +0x%llx neighbour",
                    VIEW_SPINE_STRIDE];
        return;
    }

    int runLen = 1;
    for (int step = 1; step < 512; step++) {
        uint64_t want = head + (uint64_t)step * VIEW_SPINE_STRIDE;
        bool found = false;

        for (int j = 0; j < count; j++) {
            if (cands[j].v == want) {
                found = true;
                break;
            }
        }
        if (!found)
            break;
        runLen = step + 1;
    }

    uint8_t raw[VIEW_SPINE_BYTES];
    memset(raw, 0, sizeof(raw));
    size_t have = hunt_read_obj(task, head, raw, sizeof(raw));

    /* The first qword is the class pointer on an il2cpp instance; resolving it
       names the array element so the dump can be identified directly. */
    uint64_t cls = 0;
    memcpy(&cls, raw, sizeof(cls));

    char clsName[40];
    clsName[0] = '\0';
    if (cls >= HUNT_PTR_MIN)
        class_name_at(task, cls, 0x10, meta, clsName, sizeof(clsName));

    [self vlogLine:@"VIEW SPINE: head=0x%llx run=%d nodes stride=0x%llx readable=%zu class=0x%llx name=%s",
                (unsigned long long)head, runLen,
                (unsigned long long)VIEW_SPINE_STRIDE, have,
                (unsigned long long)cls, clsName[0] ? clsName : "?"];

    for (size_t off = 0; off < sizeof(raw); off += 16) {
        size_t rowLen = sizeof(raw) - off;
        if (rowLen > 16)
            rowLen = 16;

        char hex[16 * 3 + 4];
        size_t hp = 0;
        hex[0] = '\0';
        for (size_t b = 0; b < rowLen && hp + 4 < sizeof(hex); b++)
            hp += (size_t)snprintf(hex + hp, sizeof(hex) - hp, "%02x ", raw[off + b]);

        char fl[160];
        size_t fp = 0;
        fl[0] = '\0';
        for (size_t b = 0; b + 4 <= rowLen; b += 4) {
            float f = 0.0f;
            memcpy(&f, raw + off + b, sizeof(f));
            fp += (size_t)snprintf(fl + fp, sizeof(fl) - fp, "%13.4f ", f);
        }

        [self vlogLine:@"VIEW SPINE +0x%02zx  %-48s | %s", off, hex, fl];
    }
}

/* Scans one VIEW-phase region: tags every float slot, diffs every slot against
   the previous scan, then emits changed slots (old -> new) and MAT? blocks. */
- (void)viewScanRegion:(const view_region *)reg
                  task:(mach_port_t)task
                  prev:(NSDictionary *)prev
                  next:(NSMutableDictionary *)next
                 probe:(uint64_t)probe
                 stats:(view_stats *)vs
            screenLeft:(int *)left
                  skip:(uint64_t)skipAddr
{
    size_t want = reg->len;
    if (want < 16)
        return;

    /* The T+0xb0 table gets its own snapshot and its own screen block. */
    if (skipAddr && reg->addr == skipAddr) {
        [self vlogLine:@"VIEW REG %s base=0x%llx skipped (TABLE block)",
                    reg->path, (unsigned long long)reg->addr];
        return;
    }

    uint8_t *raw = (uint8_t *)malloc(want);
    float *fv = (float *)malloc(want);
    uint8_t *tag = (uint8_t *)calloc(want / 4, 1);
    uint8_t *chg = (uint8_t *)calloc(want / 4, 1);

    if (!raw || !fv || !tag || !chg) {
        free(raw);
        free(fv);
        free(tag);
        free(chg);
        return;
    }

    size_t have = hunt_read_obj(task, reg->addr, raw, want);
    int nslots = (int)(have / 4);

    if (nslots < 4) {
        free(raw);
        free(fv);
        free(tag);
        free(chg);
        return;
    }

    memcpy(fv, raw, (size_t)nslots * sizeof(float));

    int rpos = 0;
    int rdir = 0;
    int rmat = 0;

    for (int i = 0; i + 3 <= nslots; i++) {
        if (view_pos_ok(fv + i)) {
            rpos++;
            tag[i] |= VIEW_TAG_POS_START;
            tag[i + 1] |= VIEW_TAG_POS_MEMBER;
            tag[i + 2] |= VIEW_TAG_POS_MEMBER;
        }

        if (view_dir_ok(fv + i, NULL)) {
            rdir++;
            if (i + 3 < nslots && fv[i + 3] == 0.0f)
                tag[i] |= VIEW_TAG_DIR4_START;
            else
                tag[i] |= VIEW_TAG_DIR_START;
            tag[i + 1] |= VIEW_TAG_DIR_MEMBER;
            tag[i + 2] |= VIEW_TAG_DIR_MEMBER;
        }
    }

    for (int i = 0; i + 16 <= nslots; i++) {
        if (((reg->addr + (uint64_t)i * 4) & 0xFULL) != 0)
            continue;
        if (!view_mat_ok(fv + i))
            continue;
        rmat++;
        tag[i] |= VIEW_TAG_MAT_START;
        for (int k = 1; k < 16; k++)
            tag[i + k] |= VIEW_TAG_MAT_MEMBER;
    }

    /* MAT2: perspective projection signature, row 3 is (0,0,+/-1,0). */
    int rmat2 = 0;

    for (int i = 0; i + 16 <= nslots; i++) {
        if (((reg->addr + (uint64_t)i * 4) & 0xFULL) != 0)
            continue;
        int sign = view_proj_ok(fv + i);
        if (sign == 0)
            continue;
        rmat2++;
        tag[i] |= (sign > 0) ? VIEW_TAG_MAT2_POS : VIEW_TAG_MAT2_NEG;
        for (int k = 1; k < 16; k++)
            tag[i + k] |= VIEW_TAG_MAT_MEMBER;
    }

    /* Differential pass: one dictionary lookup per slot, no output yet. */
    int rchg = 0;

    for (int i = 0; i < nslots; i++) {
        uint32_t bits = 0;
        memcpy(&bits, raw + (size_t)i * 4, sizeof(bits));

        uint64_t slotAddr = reg->addr + (uint64_t)i * 4;
        NSNumber *key = [NSNumber numberWithUnsignedLongLong:slotAddr];
        NSNumber *oldNum = prev[key];

        if (oldNum) {
            if ([oldNum unsignedIntValue] != bits) {
                chg[i] = 1;
                rchg++;
            }
        } else {
            chg[i] = 2;
            vs->added++;
        }

        next[key] = [NSNumber numberWithUnsignedInt:bits];
    }

    vs->slots += nslots;
    vs->pos += rpos;
    vs->dir += rdir;
    vs->mat += rmat;
    vs->mat2 += rmat2;
    vs->changed += rchg;

    [self vlogLine:@"VIEW REG %s base=0x%llx len=0x%zx slots=%d pos=%d dir=%d mat=%d mat2=%d chg=%d",
                reg->path, (unsigned long long)reg->addr, have, nslots,
                rpos, rdir, rmat, rmat2, rchg];

    /* The T probe stays on screen: it is the only live view-state readout. */
    if (probe && reg->addr == probe) {
        for (int off = 0x120; off <= 0x130; off += 0x10) {
            int i = off / 4;
            if (i + 3 >= nslots)
                continue;
            [self logLine:@"VIEW probe T+0x%03x = (%.4f,%.4f,%.4f,%.4f) tag=%s",
                        off, fv[i], fv[i + 1], fv[i + 2], fv[i + 3],
                        view_tag_name(tag[i])];
        }
    }

    /* Emission pass 1: every changed slot, offset order. These come first so
       the 60-line screen budget is spent on real motion, not on static
       matrices. */
    for (int i = 0; i < nslots; i++) {
        if (gStopRequested.load())
            break;
        if (chg[i] != 1)
            continue;

        NSNumber *key = [NSNumber numberWithUnsignedLongLong:
                         reg->addr + (uint64_t)i * 4];
        uint32_t oldBits = [[prev objectForKey:key] unsignedIntValue];
        float oldF = 0.0f;
        memcpy(&oldF, &oldBits, sizeof(oldF));

        [self viewEmit:[NSString stringWithFormat:
                        @"VIEW chg %s +0x%03x  %.4f -> %.4f  [%s]",
                        reg->path, (unsigned)(i * 4), oldF, fv[i],
                        view_tag_name(tag[i])]
            screenLeft:left];
    }

    /* Emission pass 2: every MAT?/MAT2? block, changed or not, offset order. */
    for (int i = 0; i < nslots; i++) {
        if (gStopRequested.load())
            break;

        if (tag[i] & VIEW_TAG_MAT_START) {
            [self viewEmit:[NSString stringWithFormat:
                            @"VIEW MAT %s +0x%03x  %8.3f %8.3f %8.3f %8.3f | "
                            @"%8.3f %8.3f %8.3f %8.3f | "
                            @"%8.3f %8.3f %8.3f %8.3f | "
                            @"%8.3f %8.3f %8.3f %8.3f",
                            reg->path, (unsigned)(i * 4),
                            fv[i + 0], fv[i + 1], fv[i + 2], fv[i + 3],
                            fv[i + 4], fv[i + 5], fv[i + 6], fv[i + 7],
                            fv[i + 8], fv[i + 9], fv[i + 10], fv[i + 11],
                            fv[i + 12], fv[i + 13], fv[i + 14], fv[i + 15]]
                        screenLeft:left];
            continue;
        }

        if (!(tag[i] & (VIEW_TAG_MAT2_POS | VIEW_TAG_MAT2_NEG)))
            continue;

        /* MAT2 goes to the file: the projection signature stays visible even
           when the screen shows only the planetary block. */
        [self vlogLine:@"VIEW MAT2 %s +0x%03x  %8.3f %8.3f %8.3f %8.3f | "
                    @"%8.3f %8.3f %8.3f %8.3f | "
                    @"%8.3f %8.3f %8.3f %8.3f | "
                    @"%8.3f %8.3f %8.3f %8.3f  [row3 %s]",
                    reg->path, (unsigned)(i * 4),
                    fv[i + 0], fv[i + 1], fv[i + 2], fv[i + 3],
                    fv[i + 4], fv[i + 5], fv[i + 6], fv[i + 7],
                    fv[i + 8], fv[i + 9], fv[i + 10], fv[i + 11],
                    fv[i + 12], fv[i + 13], fv[i + 14], fv[i + 15],
                    (tag[i] & VIEW_TAG_MAT2_POS) ? kRow3Pos : kRow3Neg];
    }

    free(raw);
    free(fv);
    free(tag);
    free(chg);
}

/* VIEW detail is file only now; the screen keeps coverage, the T probe, the
   census totals and the planetary block. */
- (void)viewEmit:(NSString *)line screenLeft:(int *)left
{
    (void)left;
    [self vlogLine:@"%@", line];
}

/* Hex + float rows for a raw dump; file only. */
- (void)viewDumpRows:(const uint8_t *)raw have:(size_t)have label:(const char *)label
{
    int nslots = (int)(have / 4);

    for (int row = 0; row + 4 <= nslots; row += 4) {
        char hex[16 * 3 + 4];
        size_t hp = 0;
        hex[0] = '\0';
        for (int b = 0; b < 16 && hp + 4 < sizeof(hex); b++) {
            hp += (size_t)snprintf(hex + hp, sizeof(hex) - hp, "%02x ",
                                   raw[(size_t)row * 4 + b]);
        }

        float f[4];
        memcpy(f, raw + (size_t)row * 4, sizeof(f));

        [self vlogLine:@"%s %03x  %-48s | %9.4f %9.4f %9.4f %9.4f",
                    label, row * 4, hex, f[0], f[1], f[2], f[3]];
    }
}

/* Item 1: the T block. Screen shows the +0x0f0..0x180 float rows, the file gets
   all of 0x000..0x300 as hex+float rows, and every unit-length vec3 in T is
   printed to both with the following float as w so forward/right/up can be
   told apart. */
- (void)viewTBlockTask:(mach_port_t)task t:(uint64_t)tObj
{
    if (!tObj)
        return;

    uint8_t raw[VIEW_SCAN_SIZE];
    memset(raw, 0, sizeof(raw));
    size_t have = hunt_read_obj(task, tObj, raw, VIEW_SCAN_SIZE);
    int nslots = (int)(have / 4);

    [self vlogLine:@"T block T=0x%llx read=0x%zx slots=%d",
               (unsigned long long)tObj, have, nslots];
    if (nslots < 4)
        return;

    [self viewDumpRows:raw have:have label:"T"];

    float fv[VIEW_SCAN_SIZE / 4];
    memcpy(fv, raw, (size_t)nslots * sizeof(float));

    for (int off = T_ROWS_FROM; off < T_ROWS_TO; off += 16) {
        int i = off / 4;
        if (i + 3 >= nslots)
            break;
        [self vlogLine:@"T+0x%03x  %9.4f %9.4f %9.4f %9.4f",
                    off, fv[i], fv[i + 1], fv[i + 2], fv[i + 3]];
    }

    int vecs = 0;
    for (int i = 0; i + 3 < nslots; i++) {
        if (!view_dir_ok(fv + i, NULL))
            continue;
        vecs++;
        [self vlogLine:@"VEC T+0x%03x  (%9.4f,%9.4f,%9.4f, %9.4f)",
                    i * 4, fv[i], fv[i + 1], fv[i + 2], fv[i + 3]];
    }
    [self vlogLine:@"T vec3 count = %d", vecs];
}

/* Item 4: snapshot of the table hanging off T+0xb0. Full hex+float dump to the
   file every scan; the screen gets the changed-slot count and the first 20
   changes. A zero count from scan 3 on is marked FROZEN. */
- (void)viewTableTask:(mach_port_t)task
                 base:(uint64_t)base
                 meta:(uint64_t)meta
                 prev:(NSDictionary *)prev
                 next:(NSMutableDictionary *)next
                  src:(const char *)src
{
    if (!base) {
        [self vlogLine:@"table: none (T+0x%02x = 0)", TABLE_OFF_FIELD];
        return;
    }

    uint8_t raw[TABLE_SCAN_SIZE];
    memset(raw, 0, sizeof(raw));
    size_t have = hunt_read_obj(task, base, raw, TABLE_SCAN_SIZE);
    int nslots = (int)(have / 4);

    uint64_t cls = 0;
    char clsName[40];
    clsName[0] = '\0';
    if (have >= 8) {
        memcpy(&cls, raw, sizeof(cls));
        if (cls >= HUNT_PTR_MIN)
            class_name_at(task, cls, 0x10, meta, clsName, sizeof(clsName));
    }

    int chg = 0;

    for (int i = 0; i < nslots; i++) {
        uint32_t bits = 0;
        memcpy(&bits, raw + (size_t)i * 4, sizeof(bits));

        NSNumber *key = [NSNumber numberWithUnsignedLongLong:
                         base + (uint64_t)i * 4];
        NSNumber *oldNum = prev[key];

        if (oldNum && [oldNum unsignedIntValue] != bits)
            chg++;

        next[key] = [NSNumber numberWithUnsignedInt:bits];
    }

    [self vlogLine:@"table X=0x%llx src=%s read=0x%zx slots=%d class=0x%llx name=%s chg=%d",
               (unsigned long long)base, src, have, nslots,
               (unsigned long long)cls, clsName[0] ? clsName : "?", chg];

    [self viewDumpRows:raw have:have label:"TBL"];

    for (int i = 0; i < nslots; i++) {
        uint32_t bits = 0;
        memcpy(&bits, raw + (size_t)i * 4, sizeof(bits));

        NSNumber *key = [NSNumber numberWithUnsignedLongLong:
                         base + (uint64_t)i * 4];
        NSNumber *oldNum = prev[key];
        if (!oldNum)
            continue;

        uint32_t oldBits = [oldNum unsignedIntValue];
        if (oldBits == bits)
            continue;

        float oldF = 0.0f;
        float newF = 0.0f;
        memcpy(&oldF, &oldBits, sizeof(oldF));
        memcpy(&newF, &bits, sizeof(newF));

        [self vlogLine:@"  table chg +0x%03x  %.4f -> %.4f", i * 4, oldF, newF];
    }

    if (have == 0) {
        [self vlogLine:@"table: unreadable at 0x%llx", (unsigned long long)base];
    } else if (chg == 0) {
        if (_scanCount >= 3)
            [self vlogLine:@"table: FROZEN (chg=0 in scan %d)", _scanCount];
        else
            [self vlogLine:@"table: chg=0 (scan %d, baseline)", _scanCount];
    } else if (chg > TABLE_SCREEN_CHG) {
        [self vlogLine:@"table: %d more changed slots in diag_scan.log",
                    chg - TABLE_SCREEN_CHG];
    }
}

/* Item 3: census over every K1779 hit. Each actor gets one line in the file;
   its first 0x148 bytes are diffed slot by slot, changed slots are listed in the
   file, and only the totals stay on screen. */
- (void)runActorCensusTask:(mach_port_t)task
                candidates:(const cand_t *)cands
                     count:(int)count
{
    [self vlogLine:@"--- actor census ---"];

    NSDictionary *prev = _actorSlots ?: [NSDictionary dictionary];
    NSMutableDictionary *next = [NSMutableDictionary dictionaryWithCapacity:8192];
    NSMutableArray *nextBases = [NSMutableArray arrayWithCapacity:512];

    int changedActors = 0;
    int changedSlots = 0;
    int runMembers = 0;
    int scanned = 0;
    bool stopped = false;

    for (int i = 0; i < count; i++) {
        if (gStopRequested.load()) {
            stopped = true;
            break;
        }

        uint64_t v = cands[i].v;
        uint8_t raw[ACTOR_SCAN_SIZE];
        memset(raw, 0, sizeof(raw));
        size_t have = hunt_read_obj(task, v, raw, sizeof(raw));
        int nslots = (int)(have / 4);
        scanned++;

        /* Round 43: keep the distinct actor bases for the ESP stage. The class
           pointer was already validated by scan_candidates. */
        if (nextBases.count < 4096)
            [nextBases addObject:@(v)];

        int inRun = 0;
        for (int j = 0; j < count; j++) {
            if (j == i)
                continue;
            if (cands[j].v == v + ACTOR_STRIDE ||
                cands[j].v == v - ACTOR_STRIDE) {
                inRun = 1;
                break;
            }
        }
        if (inRun)
            runMembers++;

        uint32_t t0 = 0, t1 = 0, t2 = 0;
        uint64_t t5878 = rd64(task, v + 0x30);
        if (t5878) {
            mr_read32(task, t5878 + 0x10, &t0);
            mr_read32(task, t5878 + 0x14, &t1);
            mr_read32(task, t5878 + 0x18, &t2);
        }

        uint8_t chgFlags[ACTOR_SCAN_SIZE / 4];
        memset(chgFlags, 0, sizeof(chgFlags));
        int chg = 0;

        for (int s = 0; s < nslots; s++) {
            uint32_t bits = 0;
            memcpy(&bits, raw + (size_t)s * 4, sizeof(bits));

            NSNumber *key = [NSNumber numberWithUnsignedLongLong:
                             v + (uint64_t)s * 4];
            NSNumber *oldNum = prev[key];

            if (oldNum && [oldNum unsignedIntValue] != bits) {
                chgFlags[s] = 1;
                chg++;
            }

            next[key] = [NSNumber numberWithUnsignedInt:bits];
        }

        NSString *line = [NSString stringWithFormat:
                          @"actor[%d] V=0x%llx cfg=%d run=%d td5878=(%d,%d,%d) read=0x%zx chg=%d",
                          i, (unsigned long long)v, (int)cands[i].config_link,
                          inRun, (int)(int32_t)t0, (int)(int32_t)t1,
                          (int)(int32_t)t2, have, chg];

        [self vlogLine:@"%@", line];

        if (chg > 0) {
            changedActors++;
            changedSlots += chg;
        }

        for (int s = 0; s < nslots; s++) {
            if (!chgFlags[s])
                continue;

            uint32_t bits = 0;
            memcpy(&bits, raw + (size_t)s * 4, sizeof(bits));

            NSNumber *key = [NSNumber numberWithUnsignedLongLong:
                             v + (uint64_t)s * 4];
            uint32_t oldBits = [[prev objectForKey:key] unsignedIntValue];

            float oldF = 0.0f;
            float newF = 0.0f;
            memcpy(&oldF, &oldBits, sizeof(oldF));
            memcpy(&newF, &bits, sizeof(newF));

            [self vlogLine:@"census chg V=0x%llx +0x%03x  %.4f -> %.4f",
                        (unsigned long long)v, (unsigned)s * 4, oldF, newF];
        }
    }

    if (!stopped) {
        _actorSlots = next;
        _actorBases = nextBases;
    }

    [self logLine:@"census: %d actors / %d changed / %d changed slots / %d stride-0x58 members",
               scanned, changedActors, changedSlots, runMembers];

    if (stopped)
        [self logLine:@"scan stopped"];
}

/* Round 35: last-resort instance finder. Walks the planetary region list,
   looks for the Camera class pointer in the object header of an aligned word,
   and validates the object through the same native/TRS probe. Stops at the
   first object that yields a transform, so a normal scan only re-reads until
   the camera is met. Budget caps the extra reads at 160 MB. */
static bool cam_heap_scan(mach_port_t task, const planet_region *regs, int count,
                          uint64_t cls, uint64_t tfCls, float *pos, float *quat,
                          uint64_t *firstObj, uint64_t *validObj,
                          uint64_t *hitsOut, uint64_t *bytesOut)
{
    if (hitsOut)
        *hitsOut = 0;
    if (bytesOut)
        *bytesOut = 0;
    if (!regs || count <= 0 || !cls)
        return false;

    size_t budget = 384u * 1024u * 1024u;   /* above PLANET_CAP_BYTES: full coverage */
    const size_t chunkCap = 1u << 18;
    uint8_t *buf = (uint8_t *)malloc(chunkCap);

    if (!buf)
        return false;

    bool found = false;
    uint64_t hits = 0;
    uint64_t bytes = 0;

    for (int r = 0; r < count && budget && !found; r++) {
        uint64_t base = regs[r].base;
        uint64_t size = regs[r].size;

        if (!base || size < 8)
            continue;

        uint64_t off = 0;

        while (off + 8 <= size && budget && !found) {
            size_t want = (size_t)((size - off > chunkCap) ? chunkCap
                                                           : (size - off));
            want &= ~(size_t)7;

            if (!want)
                break;

            /* hunt_read_obj halves on failure, so advance by what was actually
               read. The old code always advanced by the full chunk, which
               skipped almost all of a region whose read shrank. */
            size_t got = hunt_read_obj(task, base + off, buf, want);
            size_t step = got & ~(size_t)7;

            for (size_t i = 0; i + 8 <= step; i += 8) {
                uint64_t v = 0;
                memcpy(&v, buf + i, sizeof(v));
                if (v != cls)
                    continue;

                hits++;
                uint64_t obj = base + off + i;

                if (firstObj && !*firstObj)
                    *firstObj = obj;

                if (cam_find_transform(task, obj, tfCls, pos, quat, NULL)) {
                    if (validObj)
                        *validObj = obj;
                    found = true;
                    break;
                }
            }

            size_t adv = (got < want) ? step : want;

            if (!adv)
                adv = 8;

            off += adv;
            bytes += adv;
            budget = (budget > adv) ? budget - adv : 0;
        }
    }

    free(buf);

    if (hitsOut)
        *hitsOut = hits;
    if (bytesOut)
        *bytesOut = bytes;

    return found;
}

/* Round 34/35: object header test for the Camera resolver. A strict klass
   match is Camera; a different class object whose metadata name is "Camera"
   also counts (the runtime can hand out more than one class object). */
static int cam_obj_kind(mach_port_t task, uint64_t obj, uint64_t cls,
                        uint64_t meta, uint64_t nameOff)
{
    if (obj < HUNT_PTR_MIN)
        return 0;

    uint64_t k = rd64(task, obj);
    if (!k)
        return 0;
    if (k == cls)
        return 1;

    char nm[40];
    if (class_name_at(task, k, nameOff, meta, nm, sizeof(nm)) &&
        diag_streq(nm, "Camera"))
        return 2;

    return 0;
}

/* ---------------- TRS transform-chain hunt ------------------------------ */

/* Round 34: UnityEngine.Camera::main. The practice-mode camera is not the
   config actor's view state, so the instance is resolved through metadata:
   Camera class -> static-fields block -> instance whose object header is the
   Camera class -> native object -> TRS slot -> forward from the quaternion.
   kind: 3 = static main field, 4 = Camera[] element (get_main/allCameras),
   5 = fallback instance found in the statics block or on the heap. */
- (BOOL)resolveMainCameraPos:(float *)posOut fwd:(float *)fwdOut kind:(int *)kindOut
                    regions:(const planet_region *)regs count:(int)count
{
    mach_port_t task = _task;

    if (!task || !_tidBase || !_metaBase)
        return NO;

    uint64_t cls = cam_class_find(task, _tidBase, _metaBase, _nameOff);
    if (!cls) {
        [self vlogLine:@"cam: main class not found (tid=0x%llx meta=0x%llx)",
                    _tidBase, _metaBase];
        return NO;
    }

    uint64_t tfCls = tid_class_named(task, _tidBase, TD_UNITY_TRANSFORM,
                                     _metaBase, _nameOff, "Transform", NULL);

    uint64_t direct = 0;      /* static field holding a Camera instance */
    uint64_t elem = 0;        /* Camera[] element (allCameras) */
    uint64_t loose[8];
    int nLoose = 0;
    uint64_t sfSeen = 0;

    /* Round 36: statics must be a real static-fields block, not the class object
       itself. The old probe accepted the class address (statics == class) and
       then read the class header/vtable as fields, so every candidate was junk.
       A block only counts when it actually contains a Camera instance or a
       Camera[] element; the class object and its own interior are skipped. */
    for (uint64_t off = 0x00; off <= 0x400; off += 8) {
        uint64_t sf = rd64(task, cls + off);
        if (sf < HUNT_PTR_MIN || sf > 0x0000ffffffffffffULL)
            continue;
        if (sf == cls || (sf >= cls && sf < cls + 0x1000))
            continue;

        uint8_t blk[0x800];
        memset(blk, 0, sizeof(blk));
        size_t got = hunt_read_obj(task, sf, blk, sizeof(blk));
        if (got < 8)
            continue;

        bool hit = false;

        for (size_t i = 0; i + 8 <= got; i += 8) {
            uint64_t v = 0;
            memcpy(&v, blk + i, sizeof(v));
            if (v < HUNT_PTR_MIN)
                continue;

            if (cam_obj_kind(task, v, cls, _metaBase, _nameOff)) {
                if (!direct)
                    direct = v;
                hit = true;
                continue;
            }

            /* Managed array: klass, monitor, length, bounds, elements[] at
               0x20 (0x18 on the older layout). Test both element starts. */
            for (uint64_t eo = 0x18; eo <= 0x20; eo += 8) {
                uint64_t e0 = rd64(task, v + eo);
                if (cam_obj_kind(task, e0, cls, _metaBase, _nameOff)) {
                    if (!elem)
                        elem = e0;
                    hit = true;
                }
            }

            if (nLoose < 8 && !hit)
                loose[nLoose++] = v;
        }

        if (hit && !sfSeen)
            sfSeen = sf;
    }

    struct { uint64_t inst; int kind; } tries[12];
    int nt = 0;

    if (direct)
        tries[nt].inst = direct, tries[nt++].kind = 3;
    if (elem)
        tries[nt].inst = elem, tries[nt++].kind = 4;
    for (int i = 0; i < nLoose && nt < 12; i++)
        tries[nt].inst = loose[i], tries[nt++].kind = 5;

    [self vlogLine:@"cam: class=0x%llx statics=0x%llx direct=0x%llx elem=0x%llx tried=%d",
                cls, sfSeen, direct, elem, nt];

    for (int i = 0; i < nt; i++) {
        float p[3], q[4], f[3];

        if (!cam_find_transform(task, tries[i].inst, tfCls, p, q, NULL))
            continue;
        if (!cam_fwd_from_quat(q, f))
            continue;
        if (p[0] == 0.0f && p[2] == 0.0f)
            continue;

        memcpy(posOut, p, sizeof(p));
        memcpy(fwdOut, f, sizeof(f));
        if (kindOut)
            *kindOut = tries[i].kind;

        [self vlogLine:@"cam: inst=0x%llx kind=%d P=(%.3f,%.3f,%.3f) Q=(%.3f,%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                    tries[i].inst, tries[i].kind,
                    p[0], p[1], p[2], q[0], q[1], q[2], q[3],
                    f[0], f[1], f[2]];
        return YES;
    }

    /* Round 36: heap fallback ALWAYS runs after the statics attempts and always
       reports what it did. Camera components live on the managed heap, so a
       klass-pointer match plus the native TRS probe is the robust path. */
    {
        float p[3], q[4], f[3];
        uint64_t first = 0, valid = 0, hits = 0, bytes = 0;
        bool ok = false;

        memset(p, 0, sizeof(p));
        memset(q, 0, sizeof(q));

        ok = cam_heap_scan(task, regs, count, cls, tfCls, p, q,
                           &first, &valid, &hits, &bytes);

        [self vlogLine:@"cam: heap scan regions=%d hits=%llu bytes=%llu MB first=0x%llx valid=0x%llx ok=%d",
                    count, (unsigned long long)hits,
                    (unsigned long long)(bytes / (1024ULL * 1024ULL)),
                    first, valid, ok ? 1 : 0];

        if (!ok && first) {
            uint32_t raw[8];
            memset(raw, 0, sizeof(raw));
            if (mr_read(task, first, raw, sizeof(raw))) {
                [self vlogLine:@"cam: heap cand 0x%llx raw=%08x %08x %08x %08x %08x %08x %08x %08x",
                            first, raw[0], raw[1], raw[2], raw[3],
                            raw[4], raw[5], raw[6], raw[7]];
            }
        }

        if (ok && cam_fwd_from_quat(q, f) && !(p[0] == 0.0f && p[2] == 0.0f)) {
            memcpy(posOut, p, sizeof(p));
            memcpy(fwdOut, f, sizeof(f));
            if (kindOut)
                *kindOut = 5;

            [self vlogLine:@"cam: inst=0x%llx kind=5 P=(%.3f,%.3f,%.3f) Q=(%.3f,%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                        valid, p[0], p[1], p[2], q[0], q[1], q[2], q[3],
                        f[0], f[1], f[2]];
            return YES;
        }
    }

    [self vlogLine:@"cam: main unresolved (tried=%d direct=0x%llx elem=0x%llx)",
                nt, direct, elem];
    return NO;
}

/* Round 44: 1.90.2 stores the team as a byte at actor+0x24. Read the low byte
   first (the neighbouring bytes carry unrelated data), then accept the full
   word when it is itself a small team id. Direct reads keep the value current
   even when the census slot cache is stale. */
static int actor_team_at(mach_port_t task, uint64_t v, int off)
{
    uint32_t w = 0;
    int b;

    if (!mr_read32(task, v + (uint64_t)off, &w))
        return -1;

    b = (int)(w & 0xFFu);
    if (b >= 0 && b <= 2)
        return b;
    if ((int)w >= 0 && (int)w <= 2)
        return (int)w;
    return -1;
}

/* Round 43: find a small-integer actor field that splits the actor set into
   teams. Team ids are 0/1/2 in every shipped build; a boolean or a health
   value does not satisfy "local value shared with at least one actor and
   different for at least one other, at most four distinct values". Returns
   -1 when no field qualifies, in which case the caller falls back to a
   distance-based friend rule. */
- (int)teamOffsetForActors:(mach_port_t)task
                        vs:(const uint64_t *)vs count:(int)n
                    localV:(uint64_t)localV local:(int *)teamOut
{
    int firstOff = -1, firstVal = -1, firstSame = 0, firstDiff = 0;
    int bestOff = -1, bestVal = -1, bestSame = 0, bestDiff = 0;
    int order[80];
    int no = 0;

    if (!localV || n < 2)
        return -1;

    /* Round 44: 1.90.2 keeps the team byte at 0x24 (confirmed by the field
       sweep). Probe it first so noise cannot move the selection. */
    order[no++] = 0x24;

    for (int off = 0x18; off <= 0x144; off += 4) {
        if (off != 0x24)
            order[no++] = off;
    }

    for (int oi = 0; oi < no; oi++) {
        int off = order[oi];
        int lv = actor_team_at(task, localV, off);

        if (lv < 0 || lv > 2)
            continue;

        int same = 0, diff = 0, nd = 0;
        uint8_t seen[8];
        memset(seen, 0, sizeof(seen));

        for (int i = 0; i < n; i++) {
            if (vs[i] == localV)
                continue;

            int v = actor_team_at(task, vs[i], off);
            if (v < 0)
                continue;

            if (v < 8)
                seen[v] = 1;
            if (v == lv)
                same++;
            else
                diff++;
        }

        for (int i = 0; i < 8; i++)
            if (seen[i])
                nd++;

        if (same < 1 || diff < 1 || nd < 2)
            continue;

        if (firstOff < 0) {
            firstOff = off;
            firstVal = lv;
            firstSame = same;
            firstDiff = diff;
        }

        bestOff = off;
        bestVal = lv;
        bestSame = same;
        bestDiff = diff;
        break;
    }

    if (bestOff < 0) {
        bestOff = firstOff;
        bestVal = firstVal;
        bestSame = firstSame;
        bestDiff = firstDiff;
    }

    if (bestOff >= 0) {
        *teamOut = bestVal;
        [self vlogLine:@"team: off=0x%x local=%d same=%d diff=%d actors=%d",
                    bestOff, bestVal, bestSame, bestDiff, n];
    }

    return bestOff;
}

/* Round 43: stage every live actor's view-state position. This is the root the
   bind-pose skeletons cannot provide (their chain is local space), and it is
   also where the team field is read. The eye position is dropped by 1.6 m so
   the ESP box stands on the feet. */
- (void)stageActorRootsTask:(mach_port_t)task haveCam:(BOOL)haveCam
                       camP:(const float *)camP
{
    _actorRootN = 0;

    if (!_actorBases || _actorBases.count == 0)
        return;

    uint64_t vs[256];
    float ps[256 * 3];
    short eye[256];
    int n = 0;
    int scanned = 0;
    int fallbackProbes = 0;

    for (NSNumber *bn in _actorBases) {
        if (n >= 256 || scanned >= 4000 || gStopRequested.load())
            break;
        scanned++;

        uint64_t v = [bn unsignedLongLongValue];
        uint64_t t = rd64(task, v + VIEW_OFF_FIELD);
        float p[4];
        bool fromView = false;

        memset(p, 0, sizeof(p));

        /* Round 47: a remote actor may not publish a view state. Fall back to
           its component cluster (Round 43 behaviour) but keep the team-byte
           filter below, which is what rejects corpses, props and cameras. */
        if (t >= HUNT_PTR_MIN && mr_read(task, t + 0x120, p, sizeof(p))) {
            fromView = true;
        } else if (fallbackProbes < 96) {
            float hp[3];

            fallbackProbes++;
            if (!human_probe_ptr(task, v, hp, NULL))
                continue;

            p[0] = hp[0];
            p[1] = hp[1];
            p[2] = hp[2];
            p[3] = 0.0f;
        } else {
            continue;
        }

        if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]))
            continue;
        if (fabsf(p[0]) > 3000.0f || fabsf(p[2]) > 3000.0f)
            continue;
        if (fromView) {
            if (p[1] < 0.2f || p[1] > 15.0f)
                continue;
        } else if (p[1] < -2.0f || p[1] > 6.0f) {
            continue;
        }
        if (p[0] == 0.0f && p[2] == 0.0f)
            continue;

        if (haveCam) {
            float dx = p[0] - camP[0];
            float dy = p[1] - camP[1];
            float dz = p[2] - camP[2];

            if (dx * dx + dy * dy + dz * dz > 100.0f * 100.0f)
                continue;   /* far garbage / killcam copy */
        }

        int dup = 0;
        for (int i = 0; i < n; i++) {
            float dx = p[0] - ps[i * 3 + 0];
            float dy = p[1] - ps[i * 3 + 1];
            float dz = p[2] - ps[i * 3 + 2];
            if (dx * dx + dy * dy + dz * dz < 1.0f) {
                dup = 1;
                break;
            }
        }
        if (dup)
            continue;

        vs[n] = v;
        ps[n * 3 + 0] = p[0];
        ps[n * 3 + 1] = p[1];
        ps[n * 3 + 2] = p[2];
        eye[n] = fromView ? 1 : 0;
        n++;
    }

    uint64_t localV = _localV;

    if (!localV && haveCam && n > 0) {
        float best = 1.0e9f;
        for (int i = 0; i < n; i++) {
            float dx = ps[i * 3 + 0] - camP[0];
            float dy = ps[i * 3 + 1] - camP[1];
            float dz = ps[i * 3 + 2] - camP[2];
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            if (d < best) {
                best = d;
                localV = vs[i];
            }
        }
    }

    int localTeam = 0;

    /* Round 46: team ids do not depend on the view state. Probe every actor
       base, not only the handful with a live eye, so the sweep still sees
       teammates and enemies when the view filter is strict. */
    uint64_t tv[512];
    int tn = 0;

    for (NSNumber *bn in _actorBases) {
        if (tn >= 512)
            break;
        tv[tn++] = [bn unsignedLongLongValue];
    }
    if (tn < 2 && n >= 2) {
        memcpy(tv, vs, sizeof(uint64_t) * (size_t)n);
        tn = n;
    }

    int teamOff = [self teamOffsetForActors:task vs:tv count:tn localV:localV
                                      local:&localTeam];

    _localTeam = localTeam;   /* Round 48: store for bind-pose fallback */

    /* Round 50: read local player's view state directly for fallback position.
       This works even when camera fails and local player not in vs array. */
    if (localV && teamOff >= 0) {
        uint64_t localT = rd64(task, localV + VIEW_OFF_FIELD);
        if (localT) {
            float p[4];
            if (mr_read(task, localT + 0x120, p, sizeof(p)) == sizeof(p)) {
                _localPos[0] = p[0];
                _localPos[1] = p[1] - 1.6f;  /* feet level */
                _localPos[2] = p[2];
                _localPosValid = YES;
                [self vlogLine:@"local: pos=(%.2f,%.2f,%.2f) from view state", p[0], p[1], p[2]];
            }
        }
    }

    if (teamOff >= 0) {
        int shown = 0;

        for (int i = 0; i < tn && shown < 8; i++) {
            int val = actor_team_at(task, tv[i], teamOff);
            if (val < 0)
                continue;
            shown++;
            [self vlogLine:@"team: sample V=0x%llx val=%d%s",
                        (unsigned long long)tv[i], val,
                        (tv[i] == localV) ? " LOCAL" : ""];
        }
    }

    int staged = 0;
    int friends = 0;
    int logged = 0;
    int valid = 0;

    for (int i = 0; i < n; i++) {
        uint64_t v = vs[i];
        float px = ps[i * 3 + 0];
        float py = ps[i * 3 + 1];
        float pz = ps[i * 3 + 2];
        float dist = -1.0f;
        int val = -1;
        int isFriend = 0;

        if (haveCam) {
            float dx = px - camP[0];
            float dy = py - camP[1];
            float dz = pz - camP[2];
            dist = sqrtf(dx * dx + dy * dy + dz * dz);
        }

        /* Round 44: the team byte must be a team id. Anything outside 0..2 is
           not a player, even if it had a readable view state. */
        if (teamOff >= 0) {
            val = actor_team_at(task, v, teamOff);
            if (val < 0 || val > 2)
                continue;
        }

        valid++;

        /* Round 48: store ALL actor positions/teams for bind-pose fallback. */
        if (_allActorN < 512) {
            _allActorPos[_allActorN * 3 + 0] = px;
            _allActorPos[_allActorN * 3 + 1] = py - (eye[i] ? 1.6f : 0.0f);  /* feet level */
            _allActorPos[_allActorN * 3 + 2] = pz;
            _allActorTeam[_allActorN] = (teamOff >= 0 && val >= 0 && val <= 2) ? val : -1;
            _allActorN++;
        }

        /* Round 50: capture local player's feet position for bind-pose fallback
           when camera is unavailable. */
        if (v == localV) {
            _localPos[0] = px;
            _localPos[1] = py - (eye[i] ? 1.6f : 0.0f);
            _localPos[2] = pz;
            _localPosValid = YES;
            continue;   /* the local player's own eye */
        }

        if (teamOff >= 0)
            isFriend = (val == localTeam);
        else if (dist >= 0.0f)
            isFriend = (dist < 4.0f);   /* fallback: close actor is a teammate */

        float rootY = py - (eye[i] ? 1.6f : 0.0f);

        if (dist > 30.0f) {
            if (logged < 12) {
                logged++;
                [self vlogLine:@"human: actor V=0x%llx team=%s culled=1 dist=%.1f",
                            (unsigned long long)v,
                            isFriend ? "FRIEND" : "ENEMY", dist];
            }
            continue;
        }

        /* Round 46: only in-range actors feed the root list used for skeleton
           team matching and bind=1 owner pairing. */
        if (_actorRootN < ESP_MAX_BOXES) {
            _actorRootP[_actorRootN * 3 + 0] = px;
            _actorRootP[_actorRootN * 3 + 1] = rootY;
            _actorRootP[_actorRootN * 3 + 2] = pz;
            _actorRootV[_actorRootN] = v;
            _actorRootTeam[_actorRootN] = isFriend ? 1 : 0;
            _actorRootN++;
        }

        if (isFriend) {
            friends++;
        } else if (_espMode && _espStageN < ESP_MAX_BOXES) {
            _espStageRoots[_espStageN * 3 + 0] = px;
            _espStageRoots[_espStageN * 3 + 1] = rootY;
            _espStageRoots[_espStageN * 3 + 2] = pz;
            _espStageN++;
            staged++;
        }

        if (logged < 12) {
            logged++;
            [self vlogLine:@"esp: actor V=0x%llx P=(%.2f,%.2f,%.2f) root=(%.2f,%.2f,%.2f) team=%s dist=%.1f staged=%d",
                        (unsigned long long)v, px, py, pz, px, rootY, pz,
                        isFriend ? "FRIEND" : "ENEMY", dist, staged];
        }
    }

    [self vlogLine:@"esp: actors valid=%d staged=%d friends=%d teamOff=0x%x localTeam=%d localV=0x%llx",
                valid, staged, friends, teamOff, localTeam,
                (unsigned long long)localV];
}

/* Round 47: stage mover (entity record) positions into ESP.
   Mover positions come from planetary MOVER scan — position addresses that
   changed between scans. These are bot positions (feet level), work regardless
   of render state. Appends to _espStageRoots (the snapshot buffer). */
- (void)stageMoverRootsTask:(mach_port_t)task haveCam:(BOOL)haveCam
                       camP:(const float *)camP
{
    if (!_moverRootsCached || _moverRootsCachedN == 0)
        return;

    int staged = 0;

    for (int i = 0; i < _moverRootsCachedN && _espStageN < ESP_MAX_BOXES; i++) {
        float px = _moverRootsCached[i * 3 + 0];
        float py = _moverRootsCached[i * 3 + 1];
        float pz = _moverRootsCached[i * 3 + 2];

        if (!isfinite(px) || !isfinite(py) || !isfinite(pz))
            continue;
        if (fabsf(px) > 3000.0f || fabsf(pz) > 3000.0f)
            continue;
        if (py < -2.0f || py > 50.0f)
            continue;

        if (haveCam) {
            float dx = px - camP[0];
            float dy = py - camP[1];
            float dz = pz - camP[2];
            float dist = sqrtf(dx * dx + dy * dy + dz * dz);
            if (dist > 300.0f)
                continue;
        }

        /* Mover positions are at feet level (rootY = py) */
        _espStageRoots[_espStageN * 3 + 0] = px;
        _espStageRoots[_espStageN * 3 + 1] = py;
        _espStageRoots[_espStageN * 3 + 2] = pz;
        _espStageN++;
        staged++;
    }

    [self vlogLine:@"esp: mover roots staged=%d (total esp roots=%d)", staged, _espStageN];
}

/* Round 46: binary search over the base-sorted selected regions. Returns the
   index whose [base, base+size) range holds addr, or -1. */
static int region_index_for(const planet_region *regs, int n, uint64_t addr)
{
    int lo = 0;
    int hi = n - 1;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint64_t b = regs[mid].base;

        if (addr < b) {
            hi = mid - 1;
        } else if (addr >= b + regs[mid].size) {
            lo = mid + 1;
        } else {
            return mid;
        }
    }
    return -1;
}

/* Independent of the changed blocks: walks the same region list PLANETARY A
   built and tests every absolute 0x30-aligned block for a Unity transform -
   position at +0x00, unit quaternion at +0x10, uniform scale triple at +0x20.
   Slots are grouped into arrays by 0x30-consecutiveness (gap <= 0x1000). The
   summary is on screen; the array list and the position lists go to the file,
   with the array listing capped at 4000 slots. Humanoid arrays (a player /
   bot transform chain) and projection matrices are decoded from the same
   pass and close the screen after the TRS line. */
- (void)runTrsHuntTask:(mach_port_t)task
               regions:(const planet_region *)regs
                 count:(int)count
                  camP:(const float *)camP
              haveCamP:(bool)haveCamP
                  camF:(const float *)camF
              haveCamF:(bool)haveCamF
          persistAddrs:(NSArray *)persistAddrs
{
    CFAbsoluteTime t0 = CFAbsoluteTimeGetCurrent();

    trs_slot *slots = (trs_slot *)malloc((size_t)PLANET_TRS_COLLECT * sizeof(trs_slot));
    uint8_t *chunk = (uint8_t *)malloc(PLANET_TRS_CHUNK);

    if (!slots || !chunk) {
        free(slots);
        free(chunk);
        [self logLine:@"TRS failed: out of memory"];
        return;
    }

    int nslots = 0;
    uint64_t scanned = 0;
    int reads = 0;
    int readFails = 0;
    bool capped = false;
    bool stopped = false;

    /* PROJ anchors: this scan's PERSIST blocks first, then any 4-float camera
       world position row found while the TRS pass reads the heap. */
    NSMutableArray *anchors = [NSMutableArray arrayWithCapacity:PLANET_ANCHOR_MAX];
    NSMutableSet *anchorSeen = [NSMutableSet setWithCapacity:PLANET_ANCHOR_MAX];

    for (NSNumber *n in persistAddrs) {
        if (anchors.count >= PLANET_ANCHOR_MAX)
            break;
        if ([anchorSeen containsObject:n])
            continue;
        [anchorSeen addObject:n];
        [anchors addObject:n];
    }

    /* Round 46: visit the regions that hold actor objects and hunt anchors
       first, then the rest. ESP mode stops at a byte cap; non-ESP runs still
       walk everything. */
    int *ord = NULL;
    uint8_t *hot = NULL;
    uint64_t walkCap = _espMode ? PLANET_TRS_ESP_BYTES : 0;
    uint64_t walked = 0;

    if (count > 0) {
        ord = (int *)malloc((size_t)count * sizeof(int));
        hot = (uint8_t *)calloc((size_t)count, 1);

        if (ord && hot) {
            int k = 0;

            for (NSNumber *bn in _actorBases) {
                int idx = region_index_for(regs, count,
                                           [bn unsignedLongLongValue]);
                if (idx >= 0)
                    hot[idx] = 1;
            }
            for (NSNumber *an in anchors) {
                int idx = region_index_for(regs, count,
                                           [an unsignedLongLongValue]);
                if (idx >= 0)
                    hot[idx] = 1;
            }

            for (int i = 0; i < count; i++)
                if (hot[i])
                    ord[k++] = i;
            for (int i = 0; i < count; i++)
                if (!hot[i])
                    ord[k++] = i;
        } else {
            free(ord);
            ord = NULL;
        }
        free(hot);
        hot = NULL;
    }

    for (int oi = 0; oi < count && !capped; oi++) {
        if (gStopRequested.load()) {
            stopped = true;
            break;
        }

        if (walkCap && walked >= walkCap)
            break;

        int r = ord ? ord[oi] : oi;
        uint64_t end = regs[r].base + regs[r].size;
        uint64_t p = regs[r].base;
        uint64_t mis = p % PLANET_TRS_STRIDE;

        if (mis)
            p += PLANET_TRS_STRIDE - mis;

        while (p + PLANET_TRS_STRIDE <= end) {
            if (gStopRequested.load()) {
                stopped = true;
                break;
            }

            if (walkCap && walked >= walkCap)
                break;

            uint64_t left = end - p;
            size_t want = (size_t)(left < (uint64_t)PLANET_TRS_CHUNK
                                   ? left : (uint64_t)PLANET_TRS_CHUNK);
            want -= want % (size_t)PLANET_TRS_STRIDE;
            if (want == 0)
                break;

            if (mr_read(task, p, chunk, want)) {
                reads++;
            } else {
                /* Page-level retry; zeroed stretches fail the quat test, so
                   they cannot fake a TRS slot. */
                readFails++;
                for (size_t done = 0; done < want; ) {
                    size_t sub = want - done;
                    if (sub > 0x10000)
                        sub = 0x10000;
                    if (!mr_read(task, p + done, chunk + done, sub))
                        memset(chunk + done, 0, sub);
                    done += sub;
                }
            }

            size_t nblk = want / (size_t)PLANET_TRS_STRIDE;

            for (size_t b = 0; b < nblk; b++) {
                float pos[3];
                const uint8_t *bp = chunk + b * (size_t)PLANET_TRS_STRIDE;

                if (!planet_trs_ok(bp, pos))
                    continue;

                if (nslots >= PLANET_TRS_COLLECT) {
                    capped = true;
                    break;
                }

                slots[nslots].addr = p + (uint64_t)b * PLANET_TRS_STRIDE;
                slots[nslots].px = pos[0];
                slots[nslots].py = pos[1];
                slots[nslots].pz = pos[2];
                nslots++;
            }

            /* Any (P.x, P.y, P.z, 1) row in this chunk is a camera anchor: the
               projection matrix that goes with it lives within 0x800. */
            if (haveCamP && anchors.count < PLANET_ANCHOR_MAX) {
                for (size_t off = 0; off + 16 <= want; off += 4) {
                    float w4 = 0.0f;
                    float c0 = 0.0f;
                    float c1 = 0.0f;
                    float c2 = 0.0f;

                    memcpy(&w4, chunk + off + 12, sizeof(w4));
                    if (fabsf(w4 - 1.0f) > PLANET_ANCHOR_EPS)
                        continue;

                    memcpy(&c0, chunk + off, sizeof(c0));
                    memcpy(&c1, chunk + off + 4, sizeof(c1));
                    memcpy(&c2, chunk + off + 8, sizeof(c2));

                    if (fabsf(c0 - camP[0]) > PLANET_ANCHOR_EPS ||
                        fabsf(c1 - camP[1]) > PLANET_ANCHOR_EPS ||
                        fabsf(c2 - camP[2]) > PLANET_ANCHOR_EPS)
                        continue;

                    NSNumber *ak = [NSNumber numberWithUnsignedLongLong:
                                    p + (uint64_t)off];
                    if ([anchorSeen containsObject:ak])
                        continue;

                    [anchorSeen addObject:ak];
                    [anchors addObject:ak];

                    if (anchors.count >= PLANET_ANCHOR_MAX)
                        break;
                }
            }

            scanned += want;
            walked += want;
            p += want;
        }
    }

    free(chunk);
    free(ord);

    qsort(slots, (size_t)nslots, sizeof(trs_slot), trs_cmp_addr);

    /* Group into arrays: a gap of more than 0x1000 starts a new one. */
    trs_array *arrs = (trs_array *)malloc(
        (size_t)(nslots > 0 ? nslots : 1) * sizeof(trs_array));
    int narr = 0;

    if (arrs) {
        for (int i = 0; i < nslots; i++) {
            if (narr == 0 || slots[i].addr - slots[i - 1].addr > PLANET_TRS_GAP) {
                arrs[narr].base = slots[i].addr;
                arrs[narr].count = 0;
                arrs[narr].first = i;
                narr++;
            }
            arrs[narr - 1].count++;
        }
    }

    [self logLine:@"TRS: %d slots in %d arrays", nslots, narr];

    [self vlogLine:@"trs: scanned=%.1f MB cap=%.0f MB reads=%d fails=%d collect=%d/%d %s%s elapsed=%.2f",
                (double)scanned / (1024.0 * 1024.0),
                (double)walkCap / (1024.0 * 1024.0), reads, readFails,
                nslots, PLANET_TRS_COLLECT,
                capped ? "CAPPED " : "", stopped ? "STOPPED " : "",
                CFAbsoluteTimeGetCurrent() - t0];

    if (arrs && narr > 0) {
        int listed = 0;
        int humanCand = 0;
        int humanExact = 0;
        int humanLoose = 0;
        int humanMax = 0;

        for (int i = 0; i < narr; i++) {
        }

        if (listed < nslots)
            [self vlogLine:@"trs: array listing stopped at %d of %d slots",
                        listed, nslots];

        /* The 40 densest arrays get their position list: those are the ones
           that are actually transform chains (root + bones). */
        trs_array *big = (trs_array *)malloc((size_t)narr * sizeof(trs_array));

        if (big) {
            memcpy(big, arrs, (size_t)narr * sizeof(trs_array));
            qsort(big, (size_t)narr, sizeof(trs_array), trs_cmp_count);

            for (int i = 0; i < narr && i < PLANET_TRS_ARRAYS; i++) {
                [self vlogLine:@"TRS big base=0x%llx count=%d",
                            (unsigned long long)big[i].base, big[i].count];
            }

            int listedPos = 0;

            for (int i = 0; i < narr && i < PLANET_TRS_ARRAYS; i++) {
                int start = big[i].first;
                int n = big[i].count;
                if (n > PLANET_TRS_POS)
                    n = PLANET_TRS_POS;

                for (int k = 0; k < n && listedPos < PLANET_TRS_OUT_MAX; k++) {
                    trs_slot *s = &slots[start + k];
                    [self vlogLine:@"TRS pos base=0x%llx 0x%llx (%.3f,%.3f,%.3f)",
                                (unsigned long long)big[i].base,
                                (unsigned long long)s->addr,
                                s->px, s->py, s->pz];
                    listedPos++;
                }
            }
            free(big);
        }
    }

    /* ---------------- HUMANOID arrays ----------------------------------- */

    /* A live player/bot owns one transform chain. Round 30: TDM chains are
       ~141 slots and practice chains ~69, so the floor is 60 and membership is
       decided by the head height plus the four bone gaps (human_sig_start). */
    NSMutableSet *humanNow = [NSMutableSet set];
    int humanHits = 0;
    int humanBind = 0;
    int humanCull = 0;
    int humanCand = 0;
    int humanExact = 0;
    int humanLoose = 0;
    int humanMax = 0;

    /* Round 29 ESP: one camera decision per scan, then the roots below. Both go
       into the worker-owned staging buffer; nothing the overlay reads is written
       here, so scan activity cannot blank the boxes already on screen. */
    _camKind = haveCamP ? 1 : 0;

    if (haveCamP) {
        const char *tsrc = haveCamF ? "T" : "cache";

        [self vlogLine:@"cam: inst=0x0 kind=2 src=%s P=(%.3f,%.3f,%.3f) Q=(0.000,0.000,0.000,1.000) F=(%.3f,%.3f,%.3f)",
                    tsrc, camP[0], camP[1], camP[2],
                    camF[0], camF[1], camF[2]];
        [self vlogLine:@"cam: main src=%s P=(%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                    tsrc, camP[0], camP[1], camP[2],
                    camF[0], camF[1], camF[2]];
    } else {
        [self vlogLine:@"cam: main src=none (no view state, no metadata camera)"];
    }

    /* Round 34: the view-state camera (T+0x120) is absent in practice mode, so
       fall back to UnityEngine.Camera::main through the metadata. */
    float camClassP[3];
    float camClassF[3];

    if (!haveCamP) {
        int kind = 0;
        if ([self resolveMainCameraPos:camClassP fwd:camClassF kind:&kind
                               regions:regs count:count]) {
            camP = camClassP;
            camF = camClassF;
            haveCamP = true;
            haveCamF = true;
            _camKind = kind;

            const char *csrc = kind == 4 ? "get_main" : (kind == 5 ? "fallback" : "main");

            [self vlogLine:@"cam: main src=%s P=(%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                        csrc,
                        camClassP[0], camClassP[1], camClassP[2],
                        camClassF[0], camClassF[1], camClassF[2]];
        }
    }

    if (_espMode) {
        [self espStageCamera:camP haveP:haveCamP f:camF haveF:haveCamF];

        /* Round 43: actor view-state roots come first. They cover bind-pose
           skeletons and carry the team field; the skeleton loop below only
           stages when no actor was resolvable. */
        [self stageActorRootsTask:task haveCam:haveCamP camP:camP];

        /* Round 47: mover (entity record) roots — bots work when render paused. */
        [self stageMoverRootsTask:task haveCam:haveCamP camP:camP];
    } else {
        _actorRootN = 0;
    }

    if (arrs) {
        uint64_t usedRoots[ESP_MAX_BOXES];
        int nUsedRoots = 0;
        int actorCover = (_actorRootN > 0);

        for (int i = 0; i < narr; i++) {
            if (gStopRequested.load())
                break;

            if (arrs[i].count > humanMax)
                humanMax = arrs[i].count;

            if (arrs[i].count < PLANET_HUMAN_MIN)
                continue;

            humanCand++;

            [self vlogLine:@"human: array %d has %d slots (need %d)", i, arrs[i].count, PLANET_HUMAN_MIN];

            /* slot0..slot8 as raw TRS rows: pos, quat, scale. The signature
               decides membership; the bind-pose test decides drawability. */
            uint8_t trs[PLANET_HUMAN_ROWS * 0x30];
            memset(trs, 0, sizeof(trs));
            size_t th = hunt_read_obj(task, arrs[i].base, trs,
                                      PLANET_HUMAN_ROWS * 0x30);
            int rows = (int)(th / 0x30);

            int sig = human_sig_start(trs, rows);

            if (sig >= 0) {
                humanExact++;
            } else {
                sig = human_sig_loose(trs, rows);
                if (sig >= 0)
                    humanLoose++;
            }

            if (sig < 0)
                continue;

            humanHits++;

            float rootP[3];
            float headP[3];
            memcpy(rootP, trs, sizeof(rootP));
            memcpy(headP, trs + (size_t)sig * 0x30, sizeof(headP));

            if (!isfinite(rootP[0]) || !isfinite(rootP[1]) ||
                !isfinite(rootP[2]) || !isfinite(headP[1]))
                continue;

            [self vlogLine:@"human: group head_y=%.2f sig=%d root=(%.2f,%.2f,%.2f)",
                        headP[1], sig, rootP[0], rootP[1], rootP[2]];

            BOOL bind = (rootP[0] == 0.0f && rootP[1] == 0.0f &&
                         rootP[2] == 0.0f);
            BOOL drawable = NO;
            const char *roottag = "";
            const char *rootsrc = bind ? "none" : "chain";
            uint64_t rootAddr = bind ? 0 : arrs[i].base;
            uint64_t src = 0;
            uint64_t dist = 0;
            human_owner_dbg od;
            memset(&od, 0, sizeof(od));

            if (bind) {
                /* Practice-mode bind pose: the chain rows are model space.
                   Companion row first, then the owning object probed from the
                   owner header, then the wider window, then the nearest
                   in-map transform slot collected by the TRS pass. */
                float pre[3];

                if (human_bind_pre(task, arrs[i].base, pre)) {
                    drawable = YES;
                    roottag = " root=pre";
                    rootsrc = "pre";
                } else if (human_owner_root(task, arrs[i].base, pre, &src, &od)) {
                    drawable = YES;
                    roottag = " root=owner";
                    rootsrc = "owner";
                    rootAddr = src;
                    [self vlogLine:@"human: owner hit src=0x%llx nodes=%d base=0x%llx",
                                src, od.tested,
                                (unsigned long long)arrs[i].base];
                } else {
                    [self vlogLine:@"human: owner hdr=0x%llx p=0x%llx,0x%llx,0x%llx,0x%llx cands=%d tested=%d base=0x%llx",
                                od.hdr, od.p[0], od.p[1], od.p[2], od.p[3],
                                od.ncand, od.tested,
                                (unsigned long long)arrs[i].base];

                    if (human_near_trs(task, arrs[i].base, arrs[i].count,
                                       haveCamP ? camP : NULL,
                                       usedRoots, nUsedRoots,
                                       pre, &src, &dist)) {
                        drawable = YES;
                        roottag = " root=wide";
                        rootsrc = "wide";
                        rootAddr = src;
                    } else if (human_nearest_slot(slots, nslots, arrs[i].base,
                                                  arrs[i].count,
                                                  haveCamP ? camP : NULL,
                                                  usedRoots, nUsedRoots,
                                                  pre, &src, &dist)) {
                        drawable = YES;
                        roottag = " root=slot";
                        rootsrc = "slot";
                        rootAddr = src;
                    } else {
                        /* Round 48/50: bind-pose fallback — find spatially nearest
                           actor from the FULL team scan (not just in-range).
                           Uses stored _allActorPos/_allActorTeam arrays.
                           Reference point: camera if available, else local player. */
                        float refX, refY, refZ;
                        if (haveCamP) {
                            refX = camP[0]; refY = camP[1]; refZ = camP[2];
                        } else if (_localPosValid) {
                            refX = _localPos[0]; refY = _localPos[1]; refZ = _localPos[2];
                        } else {
                            refX = refY = refZ = 0.0f;
                        }

                        float bestD2 = 100.0f;   /* 10m squared */
                        int bi = -1;

                        /* Passes: 0=enemy in range, 1=friend in range, 2=enemy any, 3=friend any */
                        for (int pass = 0; pass < 4 && bi < 0; pass++) {
                            for (int k = 0; k < _allActorN; k++) {
                                int team = _allActorTeam[k];
                                bool enemy = (team >= 0 && team != _localTeam);
                                bool inRange = false;

                                if (pass == 0 || pass == 1) {
                                    if (refX == 0.0f && refY == 0.0f && refZ == 0.0f)
                                        continue;  /* no reference point */
                                    float dx = _allActorPos[k * 3 + 0] - refX;
                                    float dy = _allActorPos[k * 3 + 1] - refY;
                                    float dz = _allActorPos[k * 3 + 2] - refZ;
                                    float ad2 = dx * dx + dy * dy + dz * dz;
                                    inRange = (ad2 <= 900.0f);  /* 30m squared */
                                    if (!inRange)
                                        continue;
                                }
                                if (pass == 0 && !enemy)
                                    continue;
                                if (pass == 1 && enemy)
                                    continue;  /* pass 1: prefer friends in range */
                                if (pass == 2 && !enemy)
                                    continue;
                                if (pass == 3 && enemy)
                                    continue;  /* pass 3: prefer friends any dist */

                                /* Check if this actor root is already used by another skeleton */
                                bool used = false;
                                for (int u = 0; u < nUsedRoots; u++) {
                                    /* We can't easily match usedRoots (addresses) to allActor indices.
                                       Skip this check for now — spatial dedupe at line 4596 handles it. */
                                }
                                if (used)
                                    continue;

                                float dx = pre[0] - _allActorPos[k * 3 + 0];
                                float dy = pre[1] - _allActorPos[k * 3 + 1];
                                float dz = pre[2] - _allActorPos[k * 3 + 2];
                                float d2 = dx * dx + dy * dy + dz * dz;

                                if (d2 < bestD2) {
                                    bestD2 = d2;
                                    bi = k;
                                }
                            }
                        }

                        if (bi >= 0) {
                            drawable = YES;
                            roottag = " root=owner";
                            rootsrc = "owner";
                            rootAddr = 0;   /* no base address for all-actor entries */
                            pre[0] = _allActorPos[bi * 3 + 0];
                            pre[1] = _allActorPos[bi * 3 + 1];
                            pre[2] = _allActorPos[bi * 3 + 2];

                            [self vlogLine:@"human: owner actor idx=%d d2=%.2f team=%d base=0x%llx",
                                        bi, bestD2, _allActorTeam[bi],
                                        (unsigned long long)arrs[i].base];
                        } else {
                            humanBind++;
                            roottag = " root=BIND";
                            [self vlogLine:@"human: bind miss base=0x%llx slots=%d used=%d cam=%d allActors=%d",
                                        (unsigned long long)arrs[i].base,
                                        arrs[i].count, nUsedRoots,
                                        haveCamP ? 1 : 0, _allActorN];
                        }
                    }
                }

                if (drawable) {
                    rootP[0] = pre[0];
                    rootP[1] = pre[1];
                    rootP[2] = pre[2];
                    if (rootAddr && nUsedRoots < ESP_MAX_BOXES)
                        usedRoots[nUsedRoots++] = rootAddr;
                }
            } else if (fabsf(rootP[0]) < 3000.0f &&
                       fabsf(rootP[2]) < 3000.0f &&
                       fabsf(rootP[1]) < 50.0f) {
                drawable = YES;
            } else {
                continue;   /* chain root outside the map: not an actor */
            }

            humanHits++;

            /* Round 44: match the skeleton to the NEAREST actor root within
               10 m. The old 3 m first-match radius failed because the actor
               row is an eye position and the skeleton root sits at the feet. */
            const char *teamtag = "?";
            float teamBest = 100.0f;   /* squared 10 m */

            if (drawable && _actorRootN > 0) {
                for (int k = 0; k < _actorRootN; k++) {
                    float dx = rootP[0] - _actorRootP[k * 3 + 0];
                    float dy = rootP[1] - _actorRootP[k * 3 + 1];
                    float dz = rootP[2] - _actorRootP[k * 3 + 2];
                    float d2 = dx * dx + dy * dy + dz * dz;

                    if (d2 <= teamBest) {
                        teamBest = d2;
                        teamtag = _actorRootTeam[k] ? "FRIEND" : "ENEMY";
                    }
                }
            }

            /* Round 43: 30 m cull until team filtering is proven. */
            float camDist = -1.0f;
            BOOL culled = NO;
            char culltag[48];

            culltag[0] = '\0';

            if (drawable && haveCamP) {
                float dx = rootP[0] - camP[0];
                float dy = rootP[1] - camP[1];
                float dz = rootP[2] - camP[2];

                camDist = sqrtf(dx * dx + dy * dy + dz * dz);
                if (camDist > 30.0f) {
                    culled = YES;
                    humanCull++;
                    snprintf(culltag, sizeof(culltag), " culled=1 dist=%.1f",
                             camDist);
                }
            }

            [self vlogLine:@"human: root=0x%llx bind=%d src=%s slots=%d team=%s dist=0x%llx P=(%.3f,%.3f,%.3f)%s",
                        rootAddr, bind ? 1 : 0, rootsrc, arrs[i].count,
                        teamtag, (unsigned long long)dist,
                        rootP[0], rootP[1], rootP[2], culltag];

            if (drawable && !culled && !actorCover && _espMode &&
                _espStageN < ESP_MAX_BOXES) {
                _espStageRoots[_espStageN * 3 + 0] = rootP[0];
                _espStageRoots[_espStageN * 3 + 1] = rootP[1];
                _espStageRoots[_espStageN * 3 + 2] = rootP[2];
                _espStageN++;
            }

            NSNumber *bkey = [NSNumber numberWithUnsignedLongLong:arrs[i].base];
            BOOL stable = [_humanPrev containsObject:bkey];
            [humanNow addObject:bkey];

            if (humanHits <= PLANET_HUMAN_MAX) {
                [self logLine:@"HUMAN base=0x%llx slots=%d root=(%.3f,%.3f,%.3f) head=(%.3f,%.3f,%.3f)%s%s",
                           (unsigned long long)arrs[i].base, arrs[i].count,
                           rootP[0], rootP[1], rootP[2],
                           headP[0], headP[1], headP[2],
                           stable ? " STABLE" : "", roottag];
            } else {
                [self vlogLine:@"HUMAN base=0x%llx slots=%d root=(%.3f,%.3f,%.3f) head=(%.3f,%.3f,%.3f)%s%s",
                            (unsigned long long)arrs[i].base, arrs[i].count,
                            rootP[0], rootP[1], rootP[2],
                            headP[0], headP[1], headP[2],
                            stable ? " STABLE" : "", roottag];
            }

            /* Owner header just before the chain. */
            uint64_t hb = (arrs[i].base > PLANET_HUMAN_OWNER)
                          ? arrs[i].base - PLANET_HUMAN_OWNER : arrs[i].base;
            uint8_t hw[PLANET_HUMAN_OWNER];
            memset(hw, 0, sizeof(hw));
            size_t hh = hunt_read_obj(task, hb, hw, PLANET_HUMAN_OWNER);

            [self vlogLine:@"HUMAN OWNER 0x%llx readable=0x%zx (for 0x%llx)",
                        (unsigned long long)hb, hh,
                        (unsigned long long)arrs[i].base];
            [self viewDumpRows:hw have:hh label:"HUMAN"];

            for (int k = 0; k < rows; k++) {
                float v[12];
                memcpy(v, trs + (size_t)k * 0x30, sizeof(v));
                [self vlogLine:@"HUMAN SLOT 0x%llx pos=(%.4f,%.4f,%.4f) q=(%.4f,%.4f,%.4f,%.4f) s=(%.4f,%.4f,%.4f)",
                            (unsigned long long)(arrs[i].base + (uint64_t)k * 0x30),
                            v[0], v[1], v[2],
                            v[4], v[5], v[6], v[7],
                            v[8], v[9], v[10]];
            }
        }
    }

    _espStageBind = _espMode ? humanBind : 0;

    [self vlogLine:@"human: %d arrays (>=%d slots, head rel %.2f..%.2f) bind=%d culled=%d / %d anchors",
                humanHits, PLANET_HUMAN_MIN, PLANET_HUMAN_HEADLO,
                PLANET_HUMAN_HEADHI, humanBind, humanCull,
                (int)anchors.count];

    _humanPrev = humanNow;

    /* Staging is complete; the snapshot is published by espCompleteScan once
       the rest of the pipeline has finished. The PROJ hunt is unaffected. */

    /* ---------------- PROJ hunt ----------------------------------------- */

    int projTotal = 0;
    int projScreen = 0;
    NSMutableSet *projSeen = [NSMutableSet set];

    for (NSNumber *an in anchors) {
        if (gStopRequested.load())
            break;

        uint64_t a = [an unsignedLongLongValue];
        uint64_t sb = (a > PLANET_PROJ_HALF) ? a - PLANET_PROJ_HALF : a;
        uint8_t *pb = (uint8_t *)malloc(PLANET_PROJ_SPAN);

        if (!pb)
            break;

        memset(pb, 0, PLANET_PROJ_SPAN);
        size_t ph = hunt_read_obj(task, sb, pb, PLANET_PROJ_SPAN);

        float pf[PLANET_PROJ_SPAN / 4];
        int pn = (int)((ph & ~(size_t)3) / 4);
        memcpy(pf, pb, (size_t)pn * sizeof(float));
        free(pb);

        int found = 0;

        for (int o = 0; o + 16 <= pn && found < PLANET_PROJ_EACH; o++) {
            if (!planet_proj_ok(pf + o))
                continue;

            uint64_t addr = sb + (uint64_t)o * 4;
            NSNumber *pk = [NSNumber numberWithUnsignedLongLong:addr];

            if ([projSeen containsObject:pk])
                continue;

            [projSeen addObject:pk];
            found++;
            projTotal++;

            float m00 = pf[o];
            float m11 = pf[o + 5];
            float m22 = pf[o + 10];
            float m23 = pf[o + 11];
            float m32 = pf[o + 14];
            float fov = 2.0f * atanf(1.0f / fabsf(m11)) * 180.0f / (float)M_PI;
            const char *layout = (fabsf(m32) >= 0.7f && fabsf(m32) <= 1.3f)
                                 ? "row" : "col";

            NSString *line = [NSString stringWithFormat:
                @"PROJ cand 0x%llx: m00=%.4f m11=%.4f m22=%.4f m23=%.4f m32=%.4f fovY=%.2f deg",
                (unsigned long long)addr, m00, m11, m22, m23, m32, fov];

            if (projScreen < PLANET_PROJ_SCREEN) {
                [self logLine:@"%@", line];
                projScreen++;
            } else {
                [self vlogLine:@"%@", line];
            }

            [self vlogLine:@"PROJ ANCHOR 0x%llx base=0x%llx rel=+0x%03x layout=%s",
                        (unsigned long long)a, (unsigned long long)sb,
                        (unsigned)(o * 4), layout];
            [self viewDumpRows:(const uint8_t *)(pf + o) have:0x40 label:"PROJ"];
        }
    }

    [self vlogLine:@"proj: %d anchors / %d candidates (%d per anchor cap)",
                (int)anchors.count, projTotal, PLANET_PROJ_EACH];

    free(arrs);
    free(slots);
}

/* ---------------- PLANETARY full-heap differential ---------------------- */

/* Two-pass, whole-heap differential. Pass A enumerates every readable+writable
   region, fingerprints each 0x20 block into a 16-bit hash and stores the hash
   snapshot (capped at 320 MB of hash arrays, largest regions dropped first).
   Pass B, later in the same scan, diffs those hashes against the snapshot the
   previous scan left behind - region for region, skipping any region whose
   base/size no longer matches - and collects the changed block addresses.
   The hash arrays are then reused in place, so exactly one snapshot is ever
   live (ring of one). Changed blocks are re-read as 0x60 windows, filtered for
   a real entity record (eye-height fractional position + non-axis unit dir,
   mirrored twins collapsed), matched against the local camera for SELF, and
   compared with the previous scan's position for MOVER. */
- (void)runPlanetaryTask:(mach_port_t)task
{
    CFAbsoluteTime t0 = CFAbsoluteTimeGetCurrent();

    planet_region *cand = (planet_region *)calloc(PLANET_REGION_MAX, sizeof(planet_region));
    if (!cand) {
        [self logLine:@"PLANETARY A failed: out of memory"];
        return;
    }

    int ncand = planet_collect(task, cand, PLANET_REGION_MAX);
    if (ncand < 0) {
        free(cand);
        [self logLine:@"scan stopped"];
        return;
    }

    if (ncand >= PLANET_REGION_MAX)
        [self vlogLine:@"planetary: region cap hit (%d); rest of the heap not enumerated",
                    PLANET_REGION_MAX];

    /* Smallest first, so the 320 MB budget is spent on many small regions
       before any single huge one, and the largest are the ones dropped. */
    qsort(cand, (size_t)ncand, sizeof(planet_region), planet_cmp_size);

    planet_region *sel = (planet_region *)calloc(PLANET_REGION_MAX, sizeof(planet_region));
    if (!sel) {
        free(cand);
        [self logLine:@"PLANETARY A failed: out of memory"];
        return;
    }

    uint64_t skippedBytes = 0;
    uint64_t skippedRegions = 0;
    uint64_t hashBytes = 0;
    int nsel = 0;

    for (int i = 0; i < ncand; i++) {
        uint64_t nblocks = cand[i].size / PLANET_BLOCK;
        uint64_t need = nblocks * sizeof(uint16_t);

        if (hashBytes + need > PLANET_CAP_BYTES) {
            skippedRegions++;
            skippedBytes += cand[i].size;
            continue;
        }

        sel[nsel++] = cand[i];
        hashBytes += need;
    }

    free(cand);

    /* Base order so the old snapshot can be binary-searched. */
    qsort(sel, (size_t)nsel, sizeof(planet_region), planet_cmp_base);

    if (skippedRegions) {
        [self vlogLine:@"planetary: skipped %llu largest regions / %llu bytes (cap %llu MB)",
                    (unsigned long long)skippedRegions,
                    (unsigned long long)skippedBytes,
                    (unsigned long long)(PLANET_CAP_BYTES / (1024ULL * 1024ULL))];
    }

    uint8_t *chunk = (uint8_t *)malloc(PLANET_HASH_CHUNK);
    uint64_t *chg = (uint64_t *)malloc((size_t)PLANET_CHANGED_MAX * sizeof(uint64_t));

    if (!chunk || !chg) {
        free(chunk);
        free(chg);
        free(sel);
        [self logLine:@"PLANETARY A failed: out of memory"];
        return;
    }

    bool havePrev = (_planetNRegs > 0 && _planetRegs != NULL);
    uint64_t totalBlocks = 0;
    uint64_t hashedBytes = 0;
    uint64_t changed = 0;
    uint64_t chgHead = 0;
    uint32_t chgStored = 0;
    int mismatched = 0;
    int unreadableChunks = 0;
    bool stopped = false;

    for (int i = 0; i < nsel; i++) {
        if (gStopRequested.load()) {
            stopped = true;
            break;
        }

        planet_region *r = &sel[i];
        r->nblocks = (uint32_t)(r->size / PLANET_BLOCK);

        /* Round 46: ESP mode does not consume the differential snapshot and
           hashing the whole selected heap doubled the scan time. The region
           list is all the TRS/humanoid pass needs. */
        if (_espMode) {
            r->hash = NULL;
            continue;
        }

        planet_region *old = havePrev ? planet_find(_planetRegs, _planetNRegs, r->base) : NULL;
        bool match = (old && old->size == r->size && old->hash &&
                      old->nblocks == r->nblocks);

        if (match) {
            /* Take the old array over and refill it in place: ring of one. */
            r->hash = old->hash;
            old->hash = NULL;
        } else {
            if (havePrev) {
                mismatched++;
                [self vlogLine:@"planetary: region 0x%llx size=0x%llx has no predecessor, skipped",
                            (unsigned long long)r->base, (unsigned long long)r->size];
            }
            r->hash = (uint16_t *)malloc((size_t)r->nblocks * sizeof(uint16_t));
        }

        if (!r->hash) {
            unreadableChunks++;
            continue;
        }

        if (!match)
            memset(r->hash, 0, (size_t)r->nblocks * sizeof(uint16_t));

        uint64_t done = 0;

        while (done < r->size) {
            if (gStopRequested.load()) {
                stopped = true;
                break;
            }

            uint64_t left = r->size - done;
            size_t want = (size_t)(left < (uint64_t)PLANET_HASH_CHUNK ? left : (uint64_t)PLANET_HASH_CHUNK);
            want &= ~(size_t)(PLANET_BLOCK - 1);
            if (want == 0)
                break;

            uint16_t *h = r->hash + (size_t)(done / PLANET_BLOCK);

            if (mr_read(task, r->base + done, chunk, want)) {
                size_t nb = want / (size_t)PLANET_BLOCK;

                for (size_t b = 0; b < nb; b++) {
                    uint16_t hv = planet_hash_block(chunk + b * (size_t)PLANET_BLOCK);

                    if (match && h[b] != hv) {
                        changed++;
                        chg[chgHead % PLANET_CHANGED_MAX] =
                            r->base + (done / PLANET_BLOCK + (uint64_t)b) * PLANET_BLOCK;
                        chgHead++;
                        if (chgStored < PLANET_CHANGED_MAX)
                            chgStored++;
                    }

                    h[b] = hv;
                }
            } else {
                /* Leave the previous hashes alone: zeroing them would report
                   every block in this chunk as changed next time. */
                unreadableChunks++;
            }

            done += want;
        }

        if (stopped)
            break;

        totalBlocks += r->nblocks;
        hashedBytes += r->size;
    }

    if (stopped) {
        /* The snapshot is half-refilled; it cannot be used as a baseline. */
        for (int i = 0; i < nsel; i++)
            free(sel[i].hash);
        free(sel);
        if (havePrev) {
            for (int i = 0; i < _planetNRegs; i++)
                free(_planetRegs[i].hash);
        }
        free(_planetRegs);
        _planetRegs = NULL;
        _planetNRegs = 0;
        free(chunk);
        free(chg);
        [self logLine:@"scan stopped"];
        return;
    }

    double elapsedA = CFAbsoluteTimeGetCurrent() - t0;

    /* Rows whose old hash was never claimed belong to regions that vanished. */
    int removed = 0;
    if (havePrev) {
        for (int i = 0; i < _planetNRegs; i++) {
            if (_planetRegs[i].hash) {
                free(_planetRegs[i].hash);
                removed++;
            }
        }
    }
    free(_planetRegs);

    [self logLine:@"PLANETARY A regions=%d blocks=%llu MB=%.1f elapsed=%.2f skippedMB=%.1f",
               nsel, (unsigned long long)totalBlocks,
               (double)hashedBytes / (1024.0 * 1024.0), elapsedA,
               (double)skippedBytes / (1024.0 * 1024.0)];

    [self vlogLine:@"planetary: hash=%.1f MB / mismatch=%d / removed=%d / unreadable=%d / %s",
                (double)hashBytes / (1024.0 * 1024.0), mismatched, removed,
                unreadableChunks, havePrev ? "diff vs previous scan" : "baseline scan"];

    free(chunk);

    /* ---------------- entity extraction over changed blocks ------------ */

    /* SELF anchor: the local camera position, read fresh from the config
       actor's view state (T+0x120). */
    uint64_t selfT = _localV ? rd64(task, _localV + VIEW_OFF_FIELD) : 0;
    float selfP[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    bool haveSelfP = false;

    if (selfT && mr_read(task, selfT + 0x120, selfP, sizeof(selfP))) {
        if (isfinite(selfP[0]) && isfinite(selfP[1]) && isfinite(selfP[2]) &&
            fabsf(selfP[0]) <= 3000.0f && fabsf(selfP[2]) <= 3000.0f &&
            fabsf(selfP[1]) <= 500.0f &&
            !(selfP[0] == 0.0f && selfP[1] == 0.0f && selfP[2] == 0.0f))
            haveSelfP = true;
    }

    /* Round 28: the camera forward vector is the next 16-byte row, T+0x130.
       Kept separate from the position test so neither can invalidate the
       other; ESP falls back to the cached forward when this read fails. */
    float selfF[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    bool haveSelfF = false;
    bool haveSelfQ = false;
    float selfQ[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    if (selfT && mr_read(task, selfT + 0x130, selfF, sizeof(selfF))) {
        float fl = sqrtf(selfF[0] * selfF[0] + selfF[1] * selfF[1] +
                         selfF[2] * selfF[2]);
        if (isfinite(selfF[0]) && isfinite(selfF[1]) && isfinite(selfF[2]) &&
            fl >= ESP_CAM_FWD_MIN && fl <= ESP_CAM_FWD_MAX)
            haveSelfF = true;

        /* Round 40: the 16-byte row after the position is a rotation
           quaternion, not a direction - its length is 1 either way, so the
           fourth component decides. Forward = q * (0,0,1). */
        memcpy(selfQ, selfF, sizeof(selfQ));

        if (haveSelfF && fabsf(selfQ[3]) > 0.05f) {
            float qf[3];

            if (cam_fwd_from_quat(selfQ, qf)) {
                /* Round 41: this quaternion's optical axis is -z (the camera
                   looks down its local -z), so q * (0,0,1) is the backward
                   vector. Negate: forward = q * (0,0,-1). */
                selfF[0] = -qf[0];
                selfF[1] = -qf[1];
                selfF[2] = -qf[2];
                selfF[3] = 0.0f;
                haveSelfQ = true;
            }
        }
    }

    if (haveSelfQ) {
        [self vlogLine:@"cam: q raw=(%.3f,%.3f,%.3f,%.3f) at T+0x130=0x%llx fwd=(%.3f,%.3f,%.3f) src=T",
                    selfQ[0], selfQ[1], selfQ[2], selfQ[3],
                    (unsigned long long)(selfT + 0x130),
                    selfF[0], selfF[1], selfF[2]];
    }

    /* Round 38: the view-state camera is the primary source, but _localV can be
       stale or unset, which silently turned ESP off. Scan the cached actor
       slots for any V whose view state yields a plausible position and a unit
       forward, and take the one nearest the previous camera. */
    int viewCands = 0;

    if (!haveSelfP || !haveSelfF) {
        uint64_t bestT = 0;
        uint64_t bestV = 0;
        float bestP[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        float bestF[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        float bestScore = 1.0e9f;

        for (NSNumber *key in _actorSlots) {
            if (viewCands >= 8000)
                break;
            viewCands++;

            uint64_t v = key.unsignedLongLongValue;
            uint64_t t = rd64(task, v + VIEW_OFF_FIELD);
            if (t < HUNT_PTR_MIN)
                continue;

            float p[4], f[4];
            if (!mr_read(task, t + 0x120, p, sizeof(p)))
                continue;
            if (!mr_read(task, t + 0x130, f, sizeof(f)))
                continue;

            float fl = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
            if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]))
                continue;
            if (fabsf(p[0]) > 3000.0f || fabsf(p[2]) > 3000.0f ||
                fabsf(p[1]) > 500.0f)
                continue;
            if (p[0] == 0.0f && p[1] == 0.0f && p[2] == 0.0f)
                continue;
            if (!isfinite(fl) || fl < ESP_CAM_FWD_MIN || fl > ESP_CAM_FWD_MAX)
                continue;

            if (fabsf(f[3]) > 0.05f) {
                float qf[3];

                if (cam_fwd_from_quat(f, qf)) {
                    /* Round 41: same -z optical axis convention as the direct
                       view-state read. */
                    f[0] = -qf[0];
                    f[1] = -qf[1];
                    f[2] = -qf[2];
                    f[3] = 0.0f;
                }
            }

            float score = 0.0f;
            if (_prevSelfValid) {
                float dx = p[0] - _prevSelfP[0];
                float dy = p[1] - _prevSelfP[1];
                float dz = p[2] - _prevSelfP[2];
                score = sqrtf(dx * dx + dy * dy + dz * dz);
            }

            if (score < bestScore) {
                bestScore = score;
                bestT = t;
                bestV = v;
                memcpy(bestP, p, sizeof(bestP));
                memcpy(bestF, f, sizeof(bestF));
            }
        }

        if (bestT) {
            selfT = bestT;
            memcpy(selfP, bestP, sizeof(selfP));
            memcpy(selfF, bestF, sizeof(selfF));
            haveSelfP = YES;
            haveSelfF = YES;

            [self vlogLine:@"cam: view state T=0x%llx V=0x%llx src=scan cands=%d P=(%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                        (unsigned long long)selfT, (unsigned long long)bestV,
                        viewCands, selfP[0], selfP[1], selfP[2],
                        selfF[0], selfF[1], selfF[2]];
        } else {
            [self vlogLine:@"cam: view state missing localV=0x%llx actors=%d",
                        (unsigned long long)_localV, viewCands];
        }
    } else {
        [self vlogLine:@"cam: view state T=0x%llx src=T P=(%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f)",
                    (unsigned long long)selfT, selfP[0], selfP[1], selfP[2],
                    selfF[0], selfF[1], selfF[2]];
    }

    [self vlogLine:@"planetary: self V=0x%llx T=0x%llx P=(%.3f,%.3f,%.3f) F=(%.3f,%.3f,%.3f) %s%s",
                (unsigned long long)_localV, (unsigned long long)selfT,
                selfP[0], selfP[1], selfP[2],
                selfF[0], selfF[1], selfF[2],
                haveSelfP ? "valid" : "unavailable",
                haveSelfF ? " fwd" : " nofwd"];

    int entities = 0;
    int selfHits = 0;
    int persistHits = 0;
    int clusters = 0;
    uint64_t *entAddrs = NULL;
    int entN = 0;

    /* Value-based dedupe: the same record reached from neighbouring changed
       blocks, and the mirrored (pos, -dir) twin, collapse to one key. */
    NSMutableSet *seen = [NSMutableSet setWithCapacity:1024];

    /* Mover bookkeeping. Only candidates seen in the previous scan can move. */
    mover_rec *movedList = NULL;
    int movedN = 0;
    int movedCap = 0;
    int staticCount = 0;
    int tracked = 0;

    /* PERSIST: per-position-address SELF distance, rebuilt each scan. */
    NSMutableDictionary *selfPrevNext = [NSMutableDictionary dictionary];

    /* PERSIST addresses double as PROJ anchors for this scan. */
    NSMutableArray *persistAddrs = [NSMutableArray array];

    /* GHOST: at most PLANET_GHOST_MAX 0x200 windows, one per address. */
    NSMutableSet *ghostSeen = [NSMutableSet set];
    __block int ghostCount = 0;

    /* 0x200 hex+float window centred on addr, to the file only. */
    void (^dumpWin)(uint64_t, const char *, const char *) =
        ^(uint64_t addr, const char *label, const char *why) {
            uint64_t base = (addr > 0x100) ? addr - 0x100 : addr;
            uint8_t *w = (uint8_t *)malloc(PLANET_SELF_WIN);

            if (!w)
                return;

            memset(w, 0, PLANET_SELF_WIN);
            size_t got = hunt_read_obj(task, base, w, PLANET_SELF_WIN);

            if (why)
                [self vlogLine:@"%s WIN 0x%llx readable=0x%zx (for 0x%llx) rule=%s",
                            label, (unsigned long long)base, got,
                            (unsigned long long)addr, why];
            else
                [self vlogLine:@"%s WIN 0x%llx readable=0x%zx (for 0x%llx)",
                            label, (unsigned long long)base, got,
                            (unsigned long long)addr];

            [self viewDumpRows:w have:got label:label];
            free(w);
        };

    void (^ghostDump)(uint64_t, const char *) = ^(uint64_t addr, const char *why) {
        if (ghostCount >= PLANET_GHOST_MAX)
            return;

        NSNumber *k = [NSNumber numberWithUnsignedLongLong:addr];
        if ([ghostSeen containsObject:k])
            return;

        [ghostSeen addObject:k];
        ghostCount++;
        dumpWin(addr, "GHOST", why);
    };

    /* Deferred before/after windows: the same posAddr one scan later, so the
       layout of last scan's top movers can be compared in place. */
    if (_moverWatch.count > 0) {
        for (NSNumber *num in _moverWatch) {
            uint64_t addr = [num unsignedLongLongValue];
            uint64_t base = (addr > PLANET_WIN / 2) ? addr - PLANET_WIN / 2 : addr;
            uint8_t *w = (uint8_t *)malloc(PLANET_WIN);

            if (!w)
                break;

            memset(w, 0, PLANET_WIN);
            size_t got = hunt_read_obj(task, base, w, PLANET_WIN);
            [self vlogLine:@"MOVE AFTER 0x%llx base=0x%llx readable=0x%zx",
                        (unsigned long long)addr, (unsigned long long)base, got];
            [self viewDumpRows:w have:got label:"MAFTER"];
            free(w);
        }
        [_moverWatch removeAllObjects];
    }

    if (chgStored > 0) {
        entAddrs = (uint64_t *)malloc((size_t)chgStored * sizeof(uint64_t));
        uint8_t win[PLANET_WIN];

        for (uint32_t i = 0; i < chgStored; i++) {
            if (gStopRequested.load())
                break;

            uint64_t blockAddr = chg[(chgHead - chgStored + i) % PLANET_CHANGED_MAX];
            if (blockAddr < PLANET_TEST_BACK)
                continue;

            /* Detection window is unchanged: the 0x60 bytes starting 0x20
               before the changed block. The wider 0x100 dump is read
               separately so a region edge cannot cost us the record itself. */
            uint64_t twbase = blockAddr - PLANET_TEST_BACK;
            uint8_t twin[PLANET_TEST_LEN];
            memset(twin, 0, sizeof(twin));
            size_t have = hunt_read_obj(task, twbase, twin, PLANET_TEST_LEN);
            if (have < 16)
                continue;

            int nf = (int)((have & ~(size_t)3) / 4);
            float f[PLANET_TEST_LEN / 4];
            memcpy(f, twin, (size_t)nf * sizeof(float));

            int posOff = -1;
            for (int o = 0; o + 3 <= nf; o++) {
                if (planet_entity_pos_ok(f + o)) {
                    posOff = o;
                    break;
                }
            }
            if (posOff < 0)
                continue;

            int dirOff = -1;
            for (int o = 0; o + 3 <= nf; o++) {
                if (o >= posOff - 8 && o <= posOff + 8)
                    continue;
                if (planet_entity_dir_ok(f + o)) {
                    dirOff = o;
                    break;
                }
            }
            if (dirOff < 0)
                continue;

            float px = f[posOff];
            float py = f[posOff + 1];
            float pz = f[posOff + 2];
            float dx = f[dirOff];
            float dy = f[dirOff + 1];
            float dz = f[dirOff + 2];

            float canon[3];
            planet_canon_dir(f + dirOff, canon);

            if (seen.count < 300000) {
                NSString *key = [NSString stringWithFormat:
                                 @"%.2f/%.2f/%.2f/%.2f/%.2f/%.2f",
                                 px, py, pz, canon[0], canon[1], canon[2]];
                if ([seen containsObject:key])
                    continue;
                [seen addObject:key];
            }

            entities++;

            uint64_t posAddr = twbase + (uint64_t)posOff * 4;

            /* Wider 0x100 dump window, centred on the changed block. */
            uint64_t wbase = (blockAddr > PLANET_WIN_BACK) ? blockAddr - PLANET_WIN_BACK : blockAddr;
            memset(win, 0, sizeof(win));
            size_t whave = hunt_read_obj(task, wbase, win, PLANET_WIN);

            [self vlogLine:@"ENT 0x%llx pos=(%.3f,%.3f,%.3f) dir=(%.3f,%.3f,%.3f) posAddr=0x%llx",
                        (unsigned long long)blockAddr, px, py, pz, dx, dy, dz,
                        (unsigned long long)posAddr];
            [self vlogLine:@"ENT WIN 0x%llx pos+0x%03x dir+0x%03x",
                        (unsigned long long)wbase,
                        (unsigned)(PLANET_TEST_OFF + posOff * 4),
                        (unsigned)(PLANET_TEST_OFF + dirOff * 4)];
            [self viewDumpRows:win have:whave label:"ENT"];

            /* SELF: the candidate sitting on the local camera. Never truncated
               - this is the player-table locator. */
            if (haveSelfP) {
                float sdx = px - selfP[0];
                float sdy = py - selfP[1];
                float sdz = pz - selfP[2];
                float sd = sqrtf(sdx * sdx + sdy * sdy + sdz * sdz);

                if (sd <= PLANET_SELF_RADIUS) {
                    selfHits++;
                    [self logLine:@"SELF 0x%llx pos=(%.3f,%.3f,%.3f) dir=(%.3f,%.3f,%.3f) d=%.2f",
                               (unsigned long long)blockAddr, px, py, pz, dx, dy, dz, sd];

                    dumpWin(blockAddr, "SELF", NULL);

                    /* PERSIST: tight match now, and the same address was a
                       SELF hit in the previous scan as well. */
                    NSNumber *prevD = _selfPrev[[NSNumber numberWithUnsignedLongLong:blockAddr]];

                    if (sd <= 0.05f && prevD && [prevD floatValue] <= 0.5f) {
                        persistHits++;
                        [self logLine:@"PERSIST 0x%llx d1=%.2f d2=%.2f",
                                   (unsigned long long)blockAddr,
                                   [prevD floatValue], sd];
                        dumpWin(blockAddr, "PERSIST", NULL);
                        [persistAddrs addObject:
                         [NSNumber numberWithUnsignedLongLong:blockAddr]];
                    }

                    if (sd <= 0.5f)
                        selfPrevNext[[NSNumber numberWithUnsignedLongLong:blockAddr]] = @(sd);

                    if (blockAddr >= PLANET_GHOST_LO && blockAddr < PLANET_GHOST_HI)
                        ghostDump(blockAddr, "range");
                }
            }

            /* MOVER: compare this position address against last scan's value. */
            NSNumber *pkey = [NSNumber numberWithUnsignedLongLong:posAddr];
            NSData *oldData = _moverHist[pkey];

            if (oldData && oldData.length == sizeof(float) * 3) {
                float oldp[3];
                memcpy(oldp, oldData.bytes, sizeof(oldp));
                tracked++;

                float mdx = px - oldp[0];
                float mdy = py - oldp[1];
                float mdz = pz - oldp[2];
                float dist = sqrtf(mdx * mdx + mdy * mdy + mdz * mdz);

                /* GHOST: the mystery range, or a block that sat on the
                   previous scan's camera and then jumped well away from it. */
                if (posAddr >= PLANET_GHOST_LO && posAddr < PLANET_GHOST_HI) {
                    ghostDump(posAddr, "range");
                } else if (_prevSelfValid) {
                    float ox = oldp[0] - _prevSelfP[0];
                    float oy = oldp[1] - _prevSelfP[1];
                    float oz = oldp[2] - _prevSelfP[2];
                    float oDist = sqrtf(ox * ox + oy * oy + oz * oz);

                    float ax = px - _prevSelfP[0];
                    float ay = py - _prevSelfP[1];
                    float az = pz - _prevSelfP[2];
                    float nDist = sqrtf(ax * ax + ay * ay + az * az);

                    if (oDist <= 1.0f && nDist > 5.0f)
                        ghostDump(posAddr, "cam");
                }

                if (dist <= 0.001f) {
                    staticCount++;
                } else {
                    if (movedN == movedCap) {
                        int ncap = movedCap ? movedCap * 2 : 1024;
                        mover_rec *grown = (mover_rec *)realloc(
                            movedList, (size_t)ncap * sizeof(mover_rec));
                        if (grown) {
                            movedList = grown;
                            movedCap = ncap;
                        }
                    }
                    if (movedN < movedCap) {
                        mover_rec *mr = &movedList[movedN++];
                        mr->addr = posAddr;
                        mr->d = dist;
                        mr->ox = oldp[0];
                        mr->oy = oldp[1];
                        mr->oz = oldp[2];
                        mr->nx = px;
                        mr->ny = py;
                        mr->nz = pz;
                    }
                }
            }

            float cur[3] = { px, py, pz };
            if (!oldData)
                [_moverOrder addObject:pkey];
            _moverHist[pkey] = [NSData dataWithBytes:cur length:sizeof(cur)];

            if (entAddrs && entN < (int)chgStored)
                entAddrs[entN++] = blockAddr;
        }
    }

    /* PERSIST state for the next scan (also clears it when nothing matched). */
    _selfPrev = selfPrevNext;

    if (persistHits > 0)
        [self vlogLine:@"persist: %d address(es) hit tightly twice in a row", persistHits];

    if (ghostCount > 0)
        [self vlogLine:@"ghost: %d windows dumped (cap %d)", ghostCount, PLANET_GHOST_MAX];

    /* Ring of PLANET_MOVER_MAX position addresses; evict in batches. */
    if (_moverOrder.count > PLANET_MOVER_MAX) {
        NSUInteger drop = _moverOrder.count - PLANET_MOVER_MAX;
        if (drop < 8192)
            drop = 8192;
        if (drop > _moverOrder.count)
            drop = _moverOrder.count;

        for (NSUInteger k = 0; k < drop; k++)
            [_moverHist removeObjectForKey:_moverOrder[k]];
        [_moverOrder removeObjectsInRange:NSMakeRange(0, drop)];
    }

    if (!haveSelfP) {
        [self logLine:@"SELF: no camera P (V=0x%llx T=0x%llx)",
                   (unsigned long long)_localV, (unsigned long long)selfT];
    } else if (selfHits == 0) {
        [self logLine:@"SELF: 0 candidates within %.1f of (%.2f,%.2f,%.2f) (changed=%llu)",
                   PLANET_SELF_RADIUS, selfP[0], selfP[1], selfP[2],
                   (unsigned long long)changed];
    }

    [self logLine:@"MOVER: tracked=%d moved=%d static=%d (top %d by displacement)",
               tracked, movedN, staticCount, PLANET_SCREEN_MOVED];

    if (movedN > 0) {
        qsort(movedList, (size_t)movedN, sizeof(mover_rec), mover_cmp_desc);

        /* Keep this scan's top 20 for next scan's MOVE AFTER dump. */
        for (int i = 0; i < movedN && i < 20; i++)
            [_moverWatch addObject:
             [NSNumber numberWithUnsignedLongLong:movedList[i].addr]];

        for (int i = 0; i < movedN; i++) {
            mover_rec *mr = &movedList[i];
            NSString *line = [NSString stringWithFormat:
                              @"ENT 0x%llx moved d=%.2f (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                              (unsigned long long)mr->addr, mr->d,
                              mr->ox, mr->oy, mr->oz, mr->nx, mr->ny, mr->nz];
            if (i < PLANET_SCREEN_MOVED)
                [self logLine:@"%@", line];
            else
                [self vlogLine:@"%@", line];
        }
    }

    /* Round 47: cache mover positions for ESP (feet-level bot positions).
       These work even when render is paused because server simulates bots. */
    if (movedN > 0) {
        if (_moverRootsCached)
            free(_moverRootsCached);
        int cap = movedN < ESP_MAX_BOXES ? movedN : ESP_MAX_BOXES;
        _moverRootsCached = (float *)malloc((size_t)cap * 3 * sizeof(float));
        _moverRootsCachedN = 0;

        for (int i = 0; i < cap; i++) {
            mover_rec *mr = &movedList[i];
            _moverRootsCached[_moverRootsCachedN * 3 + 0] = mr->nx;
            _moverRootsCached[_moverRootsCachedN * 3 + 1] = mr->ny;
            _moverRootsCached[_moverRootsCachedN * 3 + 2] = mr->nz;
            _moverRootsCachedN++;
        }
    }

    free(movedList);

    /* Clusters are computed over the filtered candidates only. */
    if (entN > 1) {
        qsort(entAddrs, (size_t)entN, sizeof(uint64_t), planet_cmp_u64);

        int start = 0;
        int cluScreen = 0;

        for (int i = 1; i <= entN; i++) {
            if (i < entN && entAddrs[i] - entAddrs[i - 1] <= PLANET_CLUSTER_GAP)
                continue;

            clusters++;

            NSString *line = [NSString stringWithFormat:@"cluster base=0x%llx count=%d",
                              (unsigned long long)entAddrs[start], i - start];
            if (cluScreen < PLANET_SCREEN_CLU) {
                [self logLine:@"%@", line];
                cluScreen++;
            } else {
                [self vlogLine:@"%@", line];
            }
            start = i;
        }
    } else if (entN == 1) {
        clusters = 1;
        [self logLine:@"cluster base=0x%llx count=1",
                   (unsigned long long)entAddrs[0]];
    }

    [self logLine:@"PLANETARY B changed=%llu entities=%d clusters=%d",
               (unsigned long long)changed, entities, clusters];

    if (changed > PLANET_CHANGED_MAX) {
        [self vlogLine:@"planetary: changed ring kept last %u of %llu blocks",
                    PLANET_CHANGED_MAX, (unsigned long long)changed];
    }

    free(entAddrs);
    free(chg);

    /* TRS runs over the same region list A built, and its summary closes the
       screen for this scan. */
    if (!gStopRequested.load())
        [self runTrsHuntTask:task regions:sel count:nsel camP:selfP
                     haveCamP:haveSelfP camF:selfF haveCamF:haveSelfF
                 persistAddrs:persistAddrs];

    /* This scan's camera P becomes the GHOST reference for the next one. */
    if (haveSelfP) {
        _prevSelfP[0] = selfP[0];
        _prevSelfP[1] = selfP[1];
        _prevSelfP[2] = selfP[2];
        _prevSelfValid = YES;
    } else {
        _prevSelfValid = NO;
    }

    /* Round 29: the ESP pipeline for this scan is complete. The snapshot is
       built and swapped in here, after every read, so a later scan cannot
       blank boxes that were already drawn. */
    if (_espMode)
        [self espCompleteScan];

    /* Ring of one: this scan's hashes become the next scan's baseline. */
    _planetRegs = sel;
    _planetNRegs = nsel;
}


@end
