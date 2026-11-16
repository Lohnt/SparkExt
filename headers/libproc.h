#ifndef _LIBPROC_H_
#define _LIBPROC_H_
#include <sys/types.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

int proc_listpids(uint32_t type, uint32_t typeinfo, void *buffer, int buffersize);
int proc_listallpids(void *buffer, int buffersize);
int proc_pidpath(int pid, void *buffer, uint32_t buffersize);

#define PROC_ALL_PIDS 1
#define MAXPATHLEN 1024

#ifdef __cplusplus
}
#endif
#endif
