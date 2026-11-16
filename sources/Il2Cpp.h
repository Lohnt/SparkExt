#ifndef IL2CPP_H
#define IL2CPP_H

#include <mach/mach.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UnityFramework-relative offsets (Critical Ops 1.90.2). */
#define UF_TYPE_HANDLE_OFF    0x4E7FC78ULL
#define UF_TYPE_HANDLE_BIAS   0xC00714ULL
#define UF_METADATA_NAME_OFF  0xF1B9FULL
#define UF_DEF_TABLE_OFF      0x5086ED8ULL

#define IL2CPP_TD_ACTOR       1779
#define IL2CPP_TD_7301        7301

#define IL2CPP_ACTOR_INSTANCE_SIZE  0x148u
#define IL2CPP_ACTOR_TOKEN          0x020006F1u

typedef struct {
    uint64_t uf_base;
    uint64_t type_handle;
    uint64_t metadata_base;
    uint64_t def_table;
    uint64_t k1779;
    uint64_t k7301;
    char     name[32];
    bool     name_ok;
    bool     name_ptr_ok;
    bool     instance_size_ok;
    bool     token_ok;
} il2cpp_ctx;

bool il2cpp_bootstrap(mach_port_t task, il2cpp_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif
