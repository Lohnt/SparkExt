#ifndef MEMREAD_H
#define MEMREAD_H

#include <mach/mach.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} mr_vec3;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} mr_quat;

pid_t mr_find_pid_by_name(const char *name);
int mr_attach(pid_t pid, mach_port_t *out_task);

bool mr_read(mach_port_t task, uint64_t addr, void *buf, size_t len);
bool mr_read8(mach_port_t task, uint64_t addr, uint8_t *out);
bool mr_read32(mach_port_t task, uint64_t addr, uint32_t *out);
bool mr_read64(mach_port_t task, uint64_t addr, uint64_t *out);
bool mr_read_vec3(mach_port_t task, uint64_t addr, mr_vec3 *out);
bool mr_read_quat(mach_port_t task, uint64_t addr, mr_quat *out);
bool mr_read_cstring(mach_port_t task, uint64_t addr, char *buf, size_t maxlen);

uint64_t mr_find_unityframework_base(mach_port_t task);

#ifdef __cplusplus
}
#endif

#endif
