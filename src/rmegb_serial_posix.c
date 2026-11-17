/*
 * rmegb_serial_posix.c - serial port for Linux (termios).
 */
#if !defined(_WIN32)                    /* this file is for non-Windows systems only  */

#define _DEFAULT_SOURCE                 /* expose cfmakeraw() and CRTSCTS in glibc    */

#include "rmegb_port.h"                 /* functions implemented here                 */

#include <errno.h>                      /* errno, EINTR, EAGAIN ...                   */
#include <fcntl.h>                      /* open(), O_RDWR ...                         */
#include <poll.h>                       /* poll() for timed waits                     */
#include <stdlib.h>                     /* malloc, free                               */
#include <sys/ioctl.h>                  /* ioctl() for the latency setting            */
#include <termios.h>                    /* serial port configuration                  */
#include <time.h>                       /* clock_gettime, nanosleep                   */
#include <unistd.h>                     /* read, write, close                         */

#ifdef __linux__                        /* Linux only:                                */
#include <linux/serial.h>               /* struct serial_struct, ASYNC_LOW_LATENCY    */
#endif

typedef struct { int fd; } posix_port;  /* the handle: just the file descriptor       */

/* Map a baud number to the termios constant. */
static int to_speed(uint32_t baud, speed_t *s)
{
    switch (baud) {                                         /* supported rates only    */
    case 9600u:   *s = B9600;   return 0;                   /* 9600                    */
    case 19200u:  *s = B19200;  return 0;                   /* 19200                   */
    case 38400u:  *s = B38400;  return 0;                   /* 38400                   */
    case 57600u:  *s = B57600;  return 0;                   /* 57600                   */
    case 115200u: *s = B115200; return 0;                   /* 115200 (default)        */
    default:      return -1;                                /* anything else: refuse   */
    }
}

int port_open(void **handle, const char *name, uint32_t baud, int *os_error)
{
    struct termios tio;                                     /* port settings           */
    posix_port *p;                                          /* handle to return        */
    speed_t speed;                                          /* termios baud constant   */
    int fd;                                                 /* file descriptor         */

    *handle = NULL;                                         /* nothing open yet        */
    if (to_speed(baud, &speed) != 0) {                      /* unsupported baud rate?  */
        *os_error = EINVAL;                                 /* report "invalid"        */
        return -1;                                          /* fail                    */
    }

    fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);        /* open, not as terminal   */
    if (fd < 0) { *os_error = errno; return -1; }           /* e.g. no permission      */

    if (tcgetattr(fd, &tio) != 0) {                         /* read current settings   */
        *os_error = errno; close(fd); return -1;            /* not a serial port       */
    }
    cfmakeraw(&tio);                                        /* binary: no echo/editing */
    tio.c_cflag |= (tcflag_t)(CLOCAL | CREAD);              /* ignore modem lines, rx on*/
    tio.c_cflag &= (tcflag_t)~(CSTOPB | PARENB | CSIZE);    /* clear stop/parity/size  */
    tio.c_cflag |= CS8;                                     /* 8 data bits, N, 1       */
    tio.c_cflag &= (tcflag_t)~CRTSCTS;                      /* no hardware handshake   */
    tio.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);       /* no software handshake   */
    /* VMIN = VTIME = 0: read() never blocks; waiting is done with poll() so
       the protocol code controls the timeout exactly. */
    tio.c_cc[VMIN]  = 0;                                    /* no minimum byte count   */
    tio.c_cc[VTIME] = 0;                                    /* no inter-byte timer     */
    cfsetispeed(&tio, speed);                               /* receive baud rate       */
    cfsetospeed(&tio, speed);                               /* transmit baud rate      */
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {                /* apply the settings      */
        *os_error = errno; close(fd); return -1;            /* rejected                */
    }

#ifdef __linux__
    /* FTDI latency timer.
     * The FTDI chip in the adapter does not pass each received byte to the PC
     * at once: it waits until its 64-byte buffer fills or its latency timer
     * (16 ms by default) runs out. A gripper reply is only 8-9 bytes, so it
     * always waits the full 16 ms.
     * Without this setting every read or write takes ~16-32 ms instead of
     * ~2 ms. Things still work, just slowly; but with a short timeout_ms
     * (< ~20 ms) replies arrive too late and give RMEGB_E_TIMEOUT.
     * ASYNC_LOW_LATENCY makes the ftdi_sio driver set the timer to 1 ms.
     * If it fails (other driver, old kernel) we carry on at normal speed;
     * the timer can also be set by hand:
     *   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer */
    {
        struct serial_struct ss;                            /* driver settings         */
        if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {             /* read them               */
            ss.flags = (int)((unsigned)ss.flags |           /* add the ...             */
                             (unsigned)ASYNC_LOW_LATENCY);  /* ... low-latency flag    */
            (void)ioctl(fd, TIOCSSERIAL, &ss);              /* write back, best effort */
        }
    }
#endif

    tcflush(fd, TCIOFLUSH);                                 /* drop old buffered data  */
    p = (posix_port *)malloc(sizeof *p);                    /* allocate the handle     */
    if (!p) { *os_error = ENOMEM; close(fd); return -1; }   /* out of memory           */
    p->fd = fd;                                             /* store the descriptor    */
    *handle = p;                                            /* hand it to the caller   */
    return 0;                                               /* success                 */
}

void port_close(void *handle)
{
    posix_port *p = (posix_port *)handle;                   /* our handle type         */
    close(p->fd);                                           /* close the port          */
    free(p);                                                /* free the handle         */
}

int port_write(void *handle, const uint8_t *data, size_t len)
{
    posix_port *p = (posix_port *)handle;                   /* our handle type         */
    size_t off = 0;                                         /* bytes written so far    */

    while (off < len) {                                     /* until all is written    */
        ssize_t w = write(p->fd, data + off, len - off);    /* write the rest          */
        if (w > 0) { off += (size_t)w; continue; }          /* progress: go on         */
        if (w < 0 && errno == EINTR) continue;              /* interrupted: retry      */
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {  /* buffer full:     */
            struct pollfd pf;                               /* wait for space ...      */
            pf.fd = p->fd; pf.events = POLLOUT; pf.revents = 0;  /* ... to write       */
            if (poll(&pf, 1, 1000) <= 0) return -1;         /* none within 1 s: fail   */
            continue;                                       /* space now: retry        */
        }
        return -1;                                          /* real write error        */
    }

    /* Wait until the frame has actually left the adapter.
     * write() returns as soon as the bytes are in the kernel buffer, before
     * they are on the wire. The reply timeout should start when the gripper
     * has received the whole request. Without tcdrain() the transmit time is
     * taken out of the timeout: at 115200 baud a 13-byte frame takes ~1.1 ms
     * (harmless), at 9600 baud ~14 ms, which with a short timeout_ms can give
     * false RMEGB_E_TIMEOUT errors. */
    while (tcdrain(p->fd) != 0)                             /* wait until sent         */
        if (errno != EINTR) return -1;                      /* real error: fail        */
    return 0;                                               /* frame is on the wire    */
}

int port_read(void *handle, uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    posix_port *p = (posix_port *)handle;                   /* our handle type         */
    struct pollfd pf;                                       /* what to wait for        */
    ssize_t n;                                              /* bytes read              */
    int r;                                                  /* poll result             */

    pf.fd = p->fd; pf.events = POLLIN; pf.revents = 0;      /* wait for incoming data  */
    do {
        r = poll(&pf, 1, (int)timeout_ms);                  /* sleep until data/timeout*/
    } while (r < 0 && errno == EINTR);                      /* restart if interrupted  */
    if (r < 0) return -1;                                   /* poll failed             */
    if (r == 0) return 0;                                   /* timeout: nothing came   */
    if (!(pf.revents & POLLIN)) return -1;                  /* hang-up: unplugged      */

    n = read(p->fd, buf, max);                              /* take what is there      */
    if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;  /* none / error   */
    if (n == 0) return -1;                                  /* readable but empty: gone*/
    return (int)n;                                          /* bytes received          */
}

void port_flush_rx(void *handle)
{
    tcflush(((posix_port *)handle)->fd, TCIFLUSH);          /* drop unread input       */
}

/* CLOCK_MONOTONIC, not the wall clock: an NTP or manual clock change must
   not make a timeout expire early or last for hours. */
uint32_t port_millis(void)
{
    struct timespec ts;                                     /* seconds + nanoseconds   */
    clock_gettime(CLOCK_MONOTONIC, &ts);                    /* read the clock          */
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u +         /* seconds -> ms           */
                      (uint64_t)ts.tv_nsec / 1000000u);     /* + ns -> ms              */
}

void port_sleep_ms(uint32_t ms)
{
    struct timespec req;                                    /* time to sleep           */
    req.tv_sec  = (time_t)(ms / 1000u);                     /* whole seconds           */
    req.tv_nsec = (long)(ms % 1000u) * 1000000L;            /* remaining ms as ns      */
    while (nanosleep(&req, &req) != 0 && errno == EINTR) {  /* sleep; if interrupted,  */
    }                                                       /* sleep the rest          */
}

#else
typedef int rmegb_posix_unused;         /* ISO C forbids an empty file on Windows     */
#endif
