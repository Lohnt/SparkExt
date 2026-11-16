#import "Il2Cpp.h"
#import "MemRead.h"

#include <string.h>

bool il2cpp_bootstrap(mach_port_t task, il2cpp_ctx *ctx)
{
    if (!ctx)
        return false;

    memset(ctx, 0, sizeof(*ctx));

    ctx->uf_base = mr_find_unityframework_base(task);
    if (!ctx->uf_base)
        return false;

    if (!mr_read64(task, ctx->uf_base + UF_TYPE_HANDLE_OFF, &ctx->type_handle))
        return false;
    if (!ctx->type_handle)
        return false;

    ctx->metadata_base = ctx->type_handle - UF_TYPE_HANDLE_BIAS;

    if (!mr_read_cstring(task,
                         ctx->metadata_base + UF_METADATA_NAME_OFF,
                         ctx->name,
                         sizeof(ctx->name)))
        return false;

    ctx->name_ok = (strcmp(ctx->name, "O7D1D7DB39") == 0);

    if (!mr_read64(task, ctx->uf_base + UF_DEF_TABLE_OFF, &ctx->def_table))
        return false;

    if (!mr_read64(task, ctx->def_table + (uint64_t)IL2CPP_TD_ACTOR * 8ULL, &ctx->k1779))
        return false;

    if (!mr_read64(task, ctx->def_table + (uint64_t)IL2CPP_TD_7301 * 8ULL, &ctx->k7301))
        return false;

    uint64_t namePtr = 0;
    uint32_t instanceSize = 0;
    uint32_t token = 0;

    if (mr_read64(task, ctx->k1779 + 0x10, &namePtr))
        ctx->name_ptr_ok = (namePtr == ctx->metadata_base + UF_METADATA_NAME_OFF);

    if (mr_read32(task, ctx->k1779 + 0xF8, &instanceSize))
        ctx->instance_size_ok = (instanceSize == IL2CPP_ACTOR_INSTANCE_SIZE);

    if (mr_read32(task, ctx->k1779 + 0x11C, &token))
        ctx->token_ok = (token == IL2CPP_ACTOR_TOKEN);

    return (ctx->k1779 != 0 && ctx->k7301 != 0);
}
