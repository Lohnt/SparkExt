#import "MemRead.h"

#include "libproc.h"
#include <mach-o/loader.h>
#include <mach/mach.h>
#include <mach/vm_map.h>
#include <mach/vm_region.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const uint8_t kUnityFrameworkUUID[16] = {
    0xa8, 0x6a, 0x68, 0x9f, 0xc5, 0x2a, 0x30, 0xef,
    0xb5, 0x9f, 0x30, 0x3d, 0x93, 0xb5, 0xc7, 0x06
};

pid_t mr_find_pid_by_name(const char *name)
{
    int count = proc_listallpids(NULL, 0);
    if (count <= 0)
        return 0;

    pid_t *pids = (pid_t *)malloc(sizeof(pid_t) * (size_t)count);
    if (!pids)
        return 0;

    count = proc_listallpids(pids, (int)(sizeof(pid_t) * (size_t)count));
    if (count <= 0) {
        free(pids);
        return 0;
    }

    pid_t selfPid = getpid();
    pid_t found = 0;
    char path[MAXPATHLEN];

    for (int i = 0; i < count; i++) {
        pid_t pid = pids[i];
        if (pid <= 0 || pid == selfPid)
            continue;
        memset(path, 0, sizeof(path));
        if (proc_pidpath(pid, path, sizeof(path)) <= 0)
            continue;
        if (strstr(path, name) != NULL) {
            found = pid;
            break;
        }
    }

    free(pids);
    return found;
}

int mr_attach(pid_t pid, mach_port_t *out_task)
{
    if (!out_task)
        return -1;

    mach_port_t task = MACH_PORT_NULL;
    kern_return_t kr = task_for_pid(mach_task_self(), pid, &task);
    if (kr != KERN_SUCCESS)
        return (int)kr;

    *out_task = task;
    return 0;
}

bool mr_read(mach_port_t task, uint64_t addr, void *buf, size_t len)
{
    if (task == MACH_PORT_NULL || !buf || len == 0)
        return false;

    vm_size_t outsize = 0;
    kern_return_t kr = vm_read_overwrite(task,
                                         (vm_address_t)addr,
                                         (vm_size_t)len,
                                         (vm_address_t)buf,
                                         &outsize);
    return (kr == KERN_SUCCESS && outsize == (vm_size_t)len);
}

bool mr_read8(mach_port_t task, uint64_t addr, uint8_t *out)
{
    return mr_read(task, addr, out, sizeof(uint8_t));
}

bool mr_read32(mach_port_t task, uint64_t addr, uint32_t *out)
{
    return mr_read(task, addr, out, sizeof(uint32_t));
}

bool mr_read64(mach_port_t task, uint64_t addr, uint64_t *out)
{
    return mr_read(task, addr, out, sizeof(uint64_t));
}

bool mr_read_vec3(mach_port_t task, uint64_t addr, mr_vec3 *out)
{
    return mr_read(task, addr, out, sizeof(mr_vec3));
}

bool mr_read_quat(mach_port_t task, uint64_t addr, mr_quat *out)
{
    return mr_read(task, addr, out, sizeof(mr_quat));
}

bool mr_read_cstring(mach_port_t task, uint64_t addr, char *buf, size_t maxlen)
{
    if (!buf || maxlen == 0)
        return false;

    size_t i = 0;
    for (; i + 1 < maxlen; i++) {
        uint8_t c = 0;
        if (!mr_read8(task, addr + i, &c))
            break;
        buf[i] = (char)c;
        if (c == 0)
            return true;
    }
    buf[i] = '\0';
    return (i > 0);
}

uint64_t mr_find_unityframework_base(mach_port_t task)
{
    vm_address_t addr = 0x100000000ULL;

    for (int guard = 0; guard < 1000000; guard++) {
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

        if ((info.protection & VM_PROT_READ) && size >= sizeof(struct mach_header_64)) {
            struct mach_header_64 header;
            if (mr_read(task, addr, &header, sizeof(header)) &&
                header.magic == MH_MAGIC_64 &&
                header.sizeofcmds > 0 &&
                header.sizeofcmds <= 0x20000) {

                uint8_t *cmds = (uint8_t *)malloc(header.sizeofcmds);
                if (cmds) {
                    if (mr_read(task, addr + sizeof(header), cmds, header.sizeofcmds)) {
                        uint32_t offset = 0;
                        for (uint32_t i = 0; i < header.ncmds && offset + 8 <= header.sizeofcmds; i++) {
                            struct load_command *lc = (struct load_command *)(cmds + offset);
                            if (lc->cmdsize < 8 || offset + lc->cmdsize > header.sizeofcmds)
                                break;
                            if (lc->cmd == LC_UUID) {
                                struct uuid_command *uc = (struct uuid_command *)lc;
                                if (memcmp(uc->uuid, kUnityFrameworkUUID, sizeof(kUnityFrameworkUUID)) == 0) {
                                    free(cmds);
                                    return (uint64_t)addr;
                                }
                            }
                            offset += lc->cmdsize;
                        }
                    }
                    free(cmds);
                }
            }
        }

        vm_address_t next = addr + size;
        if (next <= addr)
            break;
        addr = next;
    }

    return 0;
}
