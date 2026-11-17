#ifndef DIAG_LOG_H
#define DIAG_LOG_H

#import <Foundation/Foundation.h>
#include <stdio.h>

/* Launch breadcrumbs.
   Each call goes to three places on purpose:
     - os_log / system log (visible in Console / idevicesyslog / TrollStore log)
     - the process stderr (captured by TrollStore's launch console)
     - an explicit fflush so a crash one instruction later cannot lose the line.
   Keeping this header-only (static inline, no new .m) means no project changes. */
static inline void diag_log(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    NSLog(@"%s", msg);
}

#endif /* DIAG_LOG_H */
