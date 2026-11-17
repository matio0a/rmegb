/*
 * rmegb_serial_win32.c - serial port for Windows 10/11 (Win32 COM API).
 */
#if defined(_WIN32)                     /* this file is for Windows only              */

#ifndef _WIN32_WINNT                    /* unless the build says otherwise ...        */
#define _WIN32_WINNT 0x0A00             /* ... target Windows 10 APIs                 */
#endif
#define WIN32_LEAN_AND_MEAN             /* leave out rarely used Windows headers      */
#include <windows.h>                    /* CreateFile, DCB, COMMTIMEOUTS, timers      */

#include "rmegb_port.h"                 /* functions implemented here                 */

#include <stdio.h>                      /* snprintf                                   */
#include <stdlib.h>                     /* malloc, free                               */
#include <string.h>                     /* memset                                     */

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION         /* older SDK headers lack it    */
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002  /* value from Windows SDK   */
#endif

typedef struct {
    HANDLE h;                           /* the open COM port                          */
    DWORD  read_timeout;                /* read timeout currently set on the port     */
} win_port;

/* Read timeouts.
 * By default ReadFile() waits until it has ALL the bytes it asked for, or
 * forever. These settings make it return as soon as at least one byte is
 * there, or after `ms` with nothing:
 *   ReadIntervalTimeout = MAXDWORD, ReadTotalTimeoutMultiplier = MAXDWORD,
 *   ReadTotalTimeoutConstant = ms.
 * ms = 0 (MAXDWORD/0/0) returns at once with whatever is buffered.
 * Without this, a missing reply hangs the program inside ReadFile(). */
static int set_read_timeout(win_port *p, DWORD ms)
{
    COMMTIMEOUTS to;                                        /* timeout settings        */
    if (p->read_timeout == ms) return 0;                    /* already set: skip       */
    memset(&to, 0, sizeof to);                              /* start from all zero     */
    to.ReadIntervalTimeout = MAXDWORD;                      /* return when data is in  */
    if (ms) {                                               /* waiting allowed:        */
        to.ReadTotalTimeoutMultiplier = MAXDWORD;           /* ... wait for 1st byte   */
        to.ReadTotalTimeoutConstant   = ms;                 /* ... at most ms          */
    }
    to.WriteTotalTimeoutConstant = 1000;                    /* never block >1 s writing*/
    if (!SetCommTimeouts(p->h, &to)) return -1;             /* apply                   */
    p->read_timeout = ms;                                   /* remember what is set    */
    return 0;                                               /* success                 */
}

int port_open(void **handle, const char *name, uint32_t baud, int *os_error)
{
    char path[64];                                          /* "\\.\COMn"              */
    win_port *p;                                            /* handle to return        */
    HANDLE h;                                               /* COM port handle         */
    DCB dcb;                                                /* port settings           */

    *handle = NULL;                                         /* nothing open yet        */
    snprintf(path, sizeof path, "\\\\.\\%s", name);         /* needed for COM10+       */
    h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE,     /* open for read+write,    */
                    0, NULL, OPEN_EXISTING,                 /* exclusively, must exist */
                    FILE_ATTRIBUTE_NORMAL, NULL);           /* plain blocking I/O      */
    if (h == INVALID_HANDLE_VALUE) {                        /* failed?                 */
        *os_error = (int)GetLastError();                    /* 2 = no such port, 5 = in use */
        return -1;                                          /* fail                    */
    }

    memset(&dcb, 0, sizeof dcb);                            /* clear settings          */
    dcb.DCBlength = sizeof dcb;                             /* required by the API     */
    if (!GetCommState(h, &dcb)) {                           /* read current settings   */
        *os_error = (int)GetLastError(); CloseHandle(h); return -1;  /* not a COM port */
    }
    dcb.BaudRate        = (DWORD)baud;                      /* baud rate               */
    dcb.ByteSize        = 8;                                /* 8 data bits             */
    dcb.Parity          = NOPARITY;                         /* no parity               */
    dcb.StopBits        = ONESTOPBIT;                       /* 1 stop bit              */
    dcb.fBinary         = TRUE;                             /* binary mode (required)  */
    dcb.fParity         = FALSE;                            /* no parity checking      */
    dcb.fOutxCtsFlow    = FALSE;                            /* ignore CTS              */
    dcb.fOutxDsrFlow    = FALSE;                            /* ignore DSR              */
    dcb.fDsrSensitivity = FALSE;                            /* receive regardless of DSR*/
    dcb.fDtrControl     = DTR_CONTROL_ENABLE;               /* DTR on, static          */
    dcb.fRtsControl     = RTS_CONTROL_ENABLE;               /* RTS on, static          */
    dcb.fOutX           = FALSE;                            /* no XON/XOFF sending     */
    dcb.fInX            = FALSE;                            /* no XON/XOFF receiving   */
    dcb.fNull           = FALSE;                            /* keep 0x00 bytes         */
    dcb.fAbortOnError   = FALSE;                            /* don't stop on errors    */
    if (!SetCommState(h, &dcb)) {                           /* apply settings          */
        *os_error = (int)GetLastError(); CloseHandle(h); return -1;  /* rejected       */
    }

    p = (win_port *)malloc(sizeof *p);                      /* allocate the handle     */
    if (!p) { *os_error = ERROR_OUTOFMEMORY; CloseHandle(h); return -1; }  /* no memory*/
    p->h = h;                                               /* store the port          */
    p->read_timeout = MAXDWORD;                             /* force first timeout set */
    if (set_read_timeout(p, 0) != 0) {                      /* non-blocking reads      */
        *os_error = (int)GetLastError();                    /* keep the reason         */
        CloseHandle(h); free(p); return -1;                 /* clean up, fail          */
    }
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR |            /* drop old buffered data  */
                 PURGE_RXABORT | PURGE_TXABORT);            /* and pending operations  */

    /* FTDI latency timer: Windows counterpart of the note in
     * rmegb_serial_posix.c. The FTDI driver holds received bytes for up to
     * 16 ms by default, so each transaction takes ~16-32 ms instead of ~2 ms,
     * and a timeout_ms below ~20 ms gives false timeouts. It cannot be set
     * through the standard COM API; set it once per adapter in Device
     * Manager > Ports > USB Serial Port > Properties > Port Settings >
     * Advanced > Latency Timer (msec) = 1. */

    *handle = p;                                            /* hand it to the caller   */
    return 0;                                               /* success                 */
}

void port_close(void *handle)
{
    win_port *p = (win_port *)handle;                       /* our handle type         */
    CloseHandle(p->h);                                      /* close the port          */
    free(p);                                                /* free the handle         */
}

int port_write(void *handle, const uint8_t *data, size_t len)
{
    win_port *p = (win_port *)handle;                       /* our handle type         */
    size_t off = 0;                                         /* bytes written so far    */
    while (off < len) {                                     /* until all is written    */
        DWORD n = 0;                                        /* bytes this call         */
        if (!WriteFile(p->h, data + off, (DWORD)(len - off), &n, NULL)  /* write rest  */
            || n == 0)                                      /* nothing went out:       */
            return -1;                                      /* error or write timeout  */
        off += n;                                           /* progress                */
    }
    /* WriteFile returns once the driver has the bytes, slightly before they
       are on the wire: transmit time is ~1.1 ms for 13 bytes at 115200 baud,
       ~14 ms at 9600, plus up to 1 ms of USB scheduling. The reply timeout
       (100 ms) absorbs that, so we do not wait here. If you run at a low baud
       rate with a much shorter timeout_ms, add FlushFileBuffers(p->h) here
       (the equivalent of tcdrain() on Linux), otherwise the transmit time is
       taken out of the timeout and you can get false RMEGB_E_TIMEOUT. */
    return 0;                                               /* success                 */
}

int port_read(void *handle, uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    win_port *p = (win_port *)handle;                       /* our handle type         */
    DWORD got = 0;                                          /* bytes received          */
    if (timeout_ms >= MAXDWORD) timeout_ms = MAXDWORD - 1u; /* MAXDWORD is not allowed */
    if (set_read_timeout(p, (DWORD)timeout_ms) != 0) return -1;  /* program the wait   */
    if (!ReadFile(p->h, buf, (DWORD)max, &got, NULL)) return -1;  /* wait and read     */
    return (int)got;                                        /* 0 = timeout             */
}

void port_flush_rx(void *handle)
{
    PurgeComm(((win_port *)handle)->h, PURGE_RXCLEAR);      /* drop unread input       */
}

/* QueryPerformanceCounter, not GetTickCount: GetTickCount only advances in
   ~15.6 ms steps, far coarser than the 2 ms inter-frame gap, which would make
   short waits and timeouts inaccurate. */
uint32_t port_millis(void)
{
    static LARGE_INTEGER freq;                              /* counter ticks per second*/
    LARGE_INTEGER c;                                        /* current counter value   */
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);  /* read once            */
    QueryPerformanceCounter(&c);                            /* read the counter        */
    return (uint32_t)((uint64_t)(c.QuadPart / freq.QuadPart) * 1000u +  /* whole s->ms */
                      (uint64_t)(c.QuadPart % freq.QuadPart) * 1000u    /* + rest, in  */
                          / (uint64_t)freq.QuadPart);       /* ms, no 64-bit overflow  */
}

/* High-resolution sleep.
 * Sleep(2) on Windows usually sleeps ~15.6 ms (the default timer tick).
 * The inter-frame gap calls this with 1-2 ms between requests, so plain
 * Sleep() would add ~15 ms to almost every transaction: a status polling
 * loop would run at ~30 Hz instead of several hundred Hz. Nothing breaks, it
 * is just slow. A high-resolution waitable timer (Windows 10 1803+) sleeps
 * close to the requested time; older systems fall back to Sleep(). */
void port_sleep_ms(uint32_t ms)
{
    static HANDLE timer = NULL;                             /* created once, kept      */
    LARGE_INTEGER due;                                      /* when the timer fires    */

    if (ms == 0) return;                                    /* nothing to wait         */
    if (!timer)                                             /* first call:             */
        timer = CreateWaitableTimerExW(NULL, NULL,          /* create a timer ...      */
                    CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,  /* ... with fine resolution*/
                    TIMER_ALL_ACCESS);                      /* ... we can set and wait */
    if (timer) {                                            /* have the timer:         */
        due.QuadPart = -(LONGLONG)ms * 10000;               /* relative, 100 ns units  */
        if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {  /* arm it          */
            WaitForSingleObject(timer, INFINITE);           /* sleep until it fires    */
            return;                                         /* done                    */
        }
    }
    Sleep(ms);                                              /* fallback: coarse sleep  */
}

#else
typedef int rmegb_win32_unused;         /* ISO C forbids an empty file on Linux       */
#endif
