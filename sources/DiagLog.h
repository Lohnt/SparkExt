#ifndef DIAG_LOG_H
#define DIAG_LOG_H

#import <Foundation/Foundation.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Persistent breadcrumb file. Appended to by every diag_log() call, so the last
   line in it is the last stage the process reached, even after a hard kill that
   leaves no .ips. main() reads it before doing anything else; the view shows it
   as "=== previous run ===". */
#define DIAG_BOOT_LOG_PATH "/var/mobile/Documents/diag_boot.log"

/* Set once in main(), before UIApplicationMain. nil if the file could not be read. */
extern NSString *gDiagPreviousRunLog;

/* Launch breadcrumbs.
   Each call goes to four places on purpose:
     - the process stderr, flushed first so a crash one instruction later cannot
       lose the line (captured by TrollStore's launch console)
     - os_log / system log (Console.app, idevicesyslog)
     - /var/mobile/Documents/diag_boot.log, appended synchronously; any error is
       silently ignored
   Keeping this header-only (static inline, no new .m) means no project changes. */
static inline void diag_log(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    NSLog(@"%s", msg);

    int fd = open(DIAG_BOOT_LOG_PATH, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd >= 0) {
        const char *p = msg;
        size_t left = strlen(msg);
        while (left > 0) {
            ssize_t n = write(fd, p, left);
            if (n <= 0)
                break;
            p += (size_t)n;
            left -= (size_t)n;
        }
        write(fd, "\n", 1);
        close(fd);
    }
}

#endif /* DIAG_LOG_H */
