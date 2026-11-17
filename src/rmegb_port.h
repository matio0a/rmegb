/*
 * rmegb_port.h - what the protocol code needs from the operating system.
 * Implemented by rmegb_serial_posix.c (Linux) and rmegb_serial_win32.c
 * (Windows). For a microcontroller, implement these functions on a UART.
 */
#ifndef RMEGB_PORT_H                    /* include guard                              */
#define RMEGB_PORT_H                    /* mark as included                           */

#include <stddef.h>                     /* size_t                                     */
#include <stdint.h>                     /* uint8_t, uint32_t                          */

/* Open and configure 8N1, no flow control. 0 = ok, -1 = failed (*os_error). */
int      port_open(void **handle, const char *name, uint32_t baud, int *os_error);
/* Close the port and free the handle. */
void     port_close(void *handle);
/* Send all bytes; return once they have left. 0 = ok, -1 = failed. */
int      port_write(void *handle, const uint8_t *data, size_t len);
/* Return as soon as >= 1 byte is there (byte count), 0 after timeout_ms
   with nothing, -1 on error. */
int      port_read(void *handle, uint8_t *buf, size_t max, uint32_t timeout_ms);
/* Throw away everything in the receive buffer. */
void     port_flush_rx(void *handle);
/* Monotonic millisecond clock; wraps after 49 days, callers use differences. */
uint32_t port_millis(void);
/* Sleep for ms milliseconds. */
void     port_sleep_ms(uint32_t ms);

#endif /* RMEGB_PORT_H */               /* end of include guard                       */
