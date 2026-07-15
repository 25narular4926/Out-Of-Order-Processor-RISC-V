/*
 * picolibc <-> port glue.
 *
 * The doomgeneric RISC-V port (stubs.h) was written for NEWLIB, whose libc calls the syscall
 * stubs `_read`, `_write`, `_sbrk`, `_close`, `_lseek`, `_fstat`, `_open`. picolibc's tinystdio
 * instead drives a single character sink through a `stdio_t` console device, and its POSIX layer
 * calls the NON-underscore names `read`/`write`/`lseek`/`close`/`open`/`fstat`.
 *
 * Rather than rewrite the port, this file bridges the two:
 *   - forwards the non-underscore POSIX names to the port's `_`-prefixed implementations, and
 *   - installs a picolibc console device that emits characters to the sim's debug-char MMIO port
 *     (0x70000000), so printf() works and we can watch Doom boot.
 *
 * stubs.h is included in exactly one translation unit (the port's), which defines the `_` stubs;
 * we declare them extern here and forward.
 */

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>

/* ---- the port's newlib-style stubs (defined in the TU that includes stubs.h) ---- */
extern ssize_t _read(int, void *, size_t);
extern ssize_t _write(int, const void *, ssize_t);
extern off_t   _lseek(int, off_t, int);
extern int     _close(int);
extern int     _open(const char *, int, mode_t);
extern int     _fstat(int, struct stat *);
extern void   *_sbrk(intptr_t);
extern int     _unlink(const char *);
extern int     _getpid(void);
extern int     _isatty(int);

/* ---- picolibc POSIX layer calls these (no underscore); forward to the port ---- */
ssize_t read(int fd, void *buf, size_t n)            { return _read(fd, buf, n); }
ssize_t write(int fd, const void *buf, size_t n)     { return _write(fd, buf, (ssize_t)n); }
off_t   lseek(int fd, off_t off, int whence)         { return _lseek(fd, off, whence); }
int     close(int fd)                                { return _close(fd); }
int     open(const char *p, int flags, ...)          { return _open(p, flags, 0); }
int     fstat(int fd, struct stat *st)               { return _fstat(fd, st); }
void   *sbrk(intptr_t inc)                           { return _sbrk(inc); }
int     unlink(const char *p)                        { return _unlink(p); }
int     getpid(void)                                 { return _getpid(); }
int     isatty(int fd)                               { return _isatty(fd); }

/* Doom renames temp savegames; we have no persistent FS, so treat rename as a no-op success. */
int     rename(const char *a, const char *b)         { (void)a; (void)b; return 0; }

/* ---- picolibc console: send stdout/stderr to the sim debug-char MMIO port ---- */
#define DBG_CHAR (*(volatile unsigned int *)0x70000000)

static int sim_putc(char c, FILE *f)
{
    (void)f;
    DBG_CHAR = (unsigned char)c;
    return c;
}

static int sim_getc(FILE *f)
{
    (void)f;
    return EOF; /* Doom never reads stdin */
}

static FILE __stdio = FDEV_SETUP_STREAM(sim_putc, sim_getc, NULL, _FDEV_SETUP_RW);
FILE *const stdin  = &__stdio;
FILE *const stdout = &__stdio;
FILE *const stderr = &__stdio;

/* ---- exit: report the code to the tohost mailbox and halt (the core cannot stop itself) ---- */
#define TOHOST (*(volatile unsigned int *)0x70000010)

void _exit(int code)
{
    TOHOST = (unsigned int)code;
    for (;;) { /* spin */ }
}

/* picolibc's abort()/assert path may call this. */
void _kill_r(void) {}
