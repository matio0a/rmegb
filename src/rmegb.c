/*
 * rmegb.c - Modbus RTU framing for RM-EGB grippers.
 */
#include "rmegb.h"                      /* API, addresses, codes, timings              */
#include "rmegb_port.h"                 /* OS serial port and timer functions         */

#include <string.h>                     /* memset, memcmp                             */

/* CRC-16/MODBUS: polynomial 0xA001 (reflected), start value 0xFFFF. */
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;                                 /* CRC start value         */
    while (len--) {                                         /* for every byte ...      */
        int i;                                              /* bit counter             */
        crc ^= *data++;                                     /* mix the byte in         */
        for (i = 0; i < 8; i++)                             /* process its 8 bits      */
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)  /* shift, apply poly */
                             : (uint16_t)(crc >> 1);        /* or just shift           */
    }
    return crc;                                             /* low byte goes out first */
}

/* Inter-frame gap.
 * Modbus RTU has no start/end characters: a frame ends when the line has
 * been silent for 3.5 character times (t3.5). The gripper uses this silence
 * to tell where one frame stops and the next begins.
 * If we start a new request sooner than t3.5 after the previous frame, the
 * gripper can glue it onto the previous one. The combined frame fails its
 * CRC, the gripper drops it silently, and we get RMEGB_E_TIMEOUT for a
 * request that was sent correctly. Most likely with back-to-back requests. */
/* The clock counts whole milliseconds, so a measured idle time of N ms can
 * really be anything from N-1 to N+1 ms: a reply ending at 1000.9 ms and a
 * check at 1001.2 ms reads as 1 ms idle although only 0.3 ms passed. We
 * therefore require one tick more than gap_ms. Without this extra tick the
 * true gap was measured as short as 1.3 ms at 115200 baud, below the 1.75 ms
 * Modbus minimum (found with the simulated gripper). */
static void wait_gap(const rmegb *g)
{
    uint32_t idle = port_millis() - g->last_ms;             /* silence so far (ms)     */
    if (idle <= g->gap_ms)                                  /* not surely long enough? */
        port_sleep_ms(g->gap_ms + 1u - idle);               /* wait until it surely is */
}

/* Send req (CRC appended here), receive a reply of resp_len bytes. */
static int exchange(rmegb *g, uint8_t *req, size_t req_len, uint8_t *resp, size_t resp_len)
{
    uint16_t crc = crc16(req, req_len);                     /* CRC of the request      */
    uint32_t start;                                         /* when we began waiting   */
    size_t got = 0;                                         /* reply bytes received    */
    size_t need = resp_len;                                 /* reply bytes expected    */

    req[req_len++] = (uint8_t)(crc & 0xFFu);                /* append CRC low byte     */
    req[req_len++] = (uint8_t)(crc >> 8);                   /* append CRC high byte    */

    wait_gap(g);                                            /* keep t3.5 silence       */

    /* Drop stale bytes before sending.
     * If an earlier request timed out, its reply may still arrive late and
     * sit in the receive buffer. Without this flush that old reply would be
     * read as the answer to THIS request: a status read could return an old
     * status, or a read could return the value of another register. */
    port_flush_rx(g->port);

    if (port_write(g->port, req, req_len) != 0) {           /* send the request        */
        g->last_ms = port_millis();                         /* bus was active now      */
        return RMEGB_E_IO;                                  /* port failed             */
    }

    if (req[0] == 0u) {                                     /* broadcast?              */
        port_sleep_ms(RMEGB_TIMEOUT_MS);                    /* turnaround, see rmegb.h */
        g->last_ms = port_millis();                         /* bus activity ends now   */
        return RMEGB_OK;                                    /* no reply to read        */
    }

    /* Reply timeout (why: see RMEGB_TIMEOUT_MS in rmegb.h). Bytes may arrive in
       several pieces, so keep reading until the whole reply is in or the
       time is up. */
    start = port_millis();                                  /* start the timeout clock */
    while (got < need) {                                    /* until reply complete    */
        uint32_t elapsed = port_millis() - start;           /* time spent so far       */
        int n;                                              /* bytes from this read    */
        if (elapsed >= g->timeout_ms) break;                /* out of time             */
        n = port_read(g->port, resp + got, need - got,      /* read what is there,     */
                      g->timeout_ms - elapsed);             /* wait at most the rest   */
        if (n < 0) {                                        /* port error              */
            g->last_ms = port_millis();                     /* bus was active now      */
            return RMEGB_E_IO;                              /* adapter gone?           */
        }
        got += (size_t)n;                                   /* count received bytes    */
        if (got >= 2 && resp[1] == (uint8_t)(req[1] | RMEGB_FC_EXCEPTION)) /* error reply?  */
            need = 5;                                       /* it is only 5 bytes long */
    }
    g->last_ms = port_millis();                             /* bus activity ends now   */

    if (got < need) return RMEGB_E_TIMEOUT;                 /* nothing or partial      */
    crc = crc16(resp, need - 2);                            /* CRC of reply body       */
    if (resp[need - 2] != (uint8_t)(crc & 0xFFu) ||         /* compare low byte        */
        resp[need - 1] != (uint8_t)(crc >> 8))              /* compare high byte       */
        return RMEGB_E_CRC;                                 /* corrupted on the wire   */
    if (resp[0] != req[0]) return RMEGB_E_FRAME;            /* answer from other station*/
    if (resp[1] == (uint8_t)(req[1] | RMEGB_FC_EXCEPTION)) { /* gripper refused?        */
        g->last_exception = resp[2];                        /* keep the reason code    */
        return RMEGB_E_EXCEPTION;                           /* report the refusal      */
    }
    if (resp[1] != req[1]) return RMEGB_E_FRAME;            /* answer to other function*/
    return RMEGB_OK;                                        /* valid reply in resp     */
}

/* Write one coil with FC05: ON is sent as FF 00, OFF as 00 00. */
static int write_coil(rmegb *g, uint16_t address, int on)
{
    uint8_t req[16];                                        /* request frame           */
    uint8_t resp[16];                                       /* reply frame             */
    int rc;                                                 /* result                  */

    req[0] = g->slave;                                      /* station number          */
    req[1] = RMEGB_FC_WRITE_COIL;                           /* function 05             */
    req[2] = (uint8_t)(address >> 8);                       /* coil address high byte  */
    req[3] = (uint8_t)(address & 0xFFu);                    /* coil address low byte   */
    req[4] = on ? 0xFFu : 0x00u;                            /* FF = on, 00 = off       */
    req[5] = 0x00u;                                         /* always 00               */
    rc = exchange(g, req, 6, resp, 8);                      /* send, reply is 8 bytes  */
    if (rc == RMEGB_OK && g->slave != 0u &&                 /* got a reply and ...     */
        memcmp(resp, req, 8) != 0)                          /* ... it is not an echo?  */
        rc = RMEGB_E_FRAME;                                 /* unexpected reply        */
    return rc;                                              /* result                  */
}

int rmegb_open(rmegb *g, const char *port, uint32_t baud, uint8_t slave)
{
    if (!g || !port) return RMEGB_E_ARG;                    /* need both               */
    memset(g, 0, sizeof *g);                                /* clear all fields        */
    if (baud == 0) baud = 115200u;                          /* 0 means gripper default */
    g->slave      = slave;                                  /* who we talk to          */
    g->timeout_ms = RMEGB_TIMEOUT_MS;                       /* reply timeout           */

    /* t3.5 = 3.5 characters x 11 bits. Above 19200 baud the Modbus spec fixes
       it at 1.75 ms. Rounded up to whole ms, the timer resolution; at 9600
       baud: 3.5 x 11 / 9600 = 4.0 ms -> 5 ms, +1 ms margin. */
    g->gap_ms = baud > 19200u ? 2u : (38500u + baud - 1u) / baud + 1u;

    if (port_open(&g->port, port, baud, &g->os_error) != 0) /* open and configure      */
        return RMEGB_E_OPEN;                                /* failed, see os_error    */
    g->last_ms = port_millis() - g->gap_ms - 1u;            /* first frame: no wait    */
    return RMEGB_OK;                                        /* ready                   */
}

void rmegb_close(rmegb *g)
{
    if (g && g->port) port_close(g->port);                  /* release the port        */
    if (g) g->port = NULL;                                  /* mark as closed          */
}

int rmegb_write(rmegb *g, uint16_t address, int32_t value)
{
    uint8_t req[16];                                        /* request frame           */
    uint8_t resp[16];                                       /* reply frame             */
    uint32_t bits;                                          /* value as raw 32 bits    */
    int rc;                                                 /* result                  */

    if (!g || !g->port) return RMEGB_E_ARG;                 /* not open                */

    /* Clamp force and position to their valid ranges: out-of-range values go
       to the nearest bound. value is signed, so negatives are seen as such. */
    if (address == RMEGB_REG_FORCE) {                       /* force: 25..100 %        */
        if (value < 25)  value = 25;                        /* negative/too low -> 25  */
        if (value > 100) value = 100;                       /* too high -> 100         */
    }
    if (address == RMEGB_REG_POSITION) {                    /* position: 0..100 %      */
        if (value < 0)   value = 0;                         /* negative -> 0           */
        if (value > 100) value = 100;                       /* too high -> 100         */
    }
    bits = (uint32_t)value;                                 /* bytes are cut from this */

    req[0]  = g->slave;                                     /* station number          */
    req[1]  = RMEGB_FC_WRITE_MULTIPLE;                      /* function 10             */
    req[2]  = (uint8_t)(address >> 8);                      /* register high byte      */
    req[3]  = (uint8_t)(address & 0xFFu);                   /* register low byte       */
    req[4]  = 0x00u;                                        /* register count high     */
    req[5]  = 0x02u;                                        /* 2 registers = 32 bits   */
    req[6]  = 0x04u;                                        /* 4 data bytes follow     */
    /* The gripper wants the LOW 16-bit word first, each word big-endian
       (manual: "swap high and low"): 50 -> 00 32 00 00,
       115200 = 0x0001C200 -> C2 00 00 01. */
    req[7]  = (uint8_t)(bits >> 8);                         /* low word, high byte     */
    req[8]  = (uint8_t)(bits);                              /* low word, low byte      */
    req[9]  = (uint8_t)(bits >> 24);                        /* high word, high byte    */
    req[10] = (uint8_t)(bits >> 16);                        /* high word, low byte     */
    rc = exchange(g, req, 11, resp, 8);                     /* send, reply is 8 bytes  */
    if (rc == RMEGB_OK && g->slave != 0u &&                 /* got a reply and ...     */
        memcmp(resp, req, 6) != 0)                          /* ... not for our register*/
        rc = RMEGB_E_FRAME;                                 /* unexpected reply        */
    return rc;                                              /* result                  */
}

int rmegb_read(rmegb *g, uint16_t address, uint32_t *value)
{
    uint8_t req[16];                                        /* request frame           */
    uint8_t resp[16];                                       /* reply frame             */
    int rc;                                                 /* result                  */

    if (!g || !g->port || !value) return RMEGB_E_ARG;       /* not open / no output    */
    if (g->slave == 0u) return RMEGB_E_ARG;                 /* broadcasts get no reply */

    req[0] = g->slave;                                      /* station number          */
    req[1] = RMEGB_FC_READ_HOLDING;                         /* function 03             */
    req[2] = (uint8_t)(address >> 8);                       /* register high byte      */
    req[3] = (uint8_t)(address & 0xFFu);                    /* register low byte       */
    req[4] = 0x00u;                                         /* register count high     */
    req[5] = 0x02u;                                         /* 2 registers = 32 bits   */
    rc = exchange(g, req, 6, resp, 9);                      /* send, reply is 9 bytes  */
    if (rc != RMEGB_OK) return rc;                          /* failed                  */
    if (resp[2] != 4u) return RMEGB_E_FRAME;                /* must carry 4 data bytes */

    *value = ((uint32_t)resp[3] << 8)  |                    /* low word, high byte     */
             (uint32_t)resp[4]         |                    /* low word, low byte      */
             ((uint32_t)resp[5] << 24) |                    /* high word, high byte    */
             ((uint32_t)resp[6] << 16);                     /* high word, low byte     */
    return RMEGB_OK;                                        /* value is valid          */
}

int rmegb_initialize(rmegb *g, uint32_t timeout_ms)
{
    uint32_t status = 0;                                    /* last status read        */
    uint32_t start;                                         /* when homing was started */
    int rc;                                                 /* result                  */

    if (!g || !g->port) return RMEGB_E_ARG;                 /* not open                */

    /* The gripper starts homing on the 0 -> 1 change of the coil, so the coil
       is first written 0, then 1. No extra delay is needed between the two:
       the gripper's reply to the first write shows it has processed it, and
       exchange() already keeps the inter-frame gap. */
    rc = write_coil(g, RMEGB_COIL_INIT, 0);                 /* coil low: arm the edge  */
    if (rc != RMEGB_OK) return rc;                          /* not accepted            */
    rc = write_coil(g, RMEGB_COIL_INIT, 1);                 /* coil high: start homing */
    if (rc != RMEGB_OK) return rc;                          /* not accepted            */
    if (g->slave == 0u) return RMEGB_OK;                    /* broadcast: can't poll   */

    start = port_millis();                                  /* homing started now      */

    /* Phase 1: wait for bit2 to clear (see RMEGB_INIT_START_WINDOW_MS). If it never
       clears within the window, the gripper finished very quickly or does
       not clear it; phase 2 then returns at its first read. */
    while (port_millis() - start < RMEGB_INIT_START_WINDOW_MS) { /* inside the window       */
        rc = rmegb_read(g, RMEGB_REG_STATUS, &status);      /* read status             */
        if (rc == RMEGB_OK && !(status & RMEGB_STATUS_INITIALIZED))  /* bit2 cleared?  */
            break;                                          /* homing is under way     */
        if (rc != RMEGB_OK && rc != RMEGB_E_TIMEOUT &&      /* not just a missed ...   */
            rc != RMEGB_E_CRC)                              /* ... or garbled reply?   */
            return rc;                                      /* real error: give up     */
        port_sleep_ms(RMEGB_STATUS_POLL_MS);                /* poll period             */
    }

    /* Phase 2: wait for bit2 to set. A timeout or CRC error on a single read
       is tolerated in both phases: the gripper may not answer while it is
       busy homing, and the motor starting up can disturb the RS-485 line.
       Without this, one lost or garbled status reply would abort a homing
       run that is actually going fine. Other errors (port failure, Modbus
       exception, reply from another station) still end the wait at once. */
    while (port_millis() - start < timeout_ms) {            /* until overall timeout   */
        rc = rmegb_read(g, RMEGB_REG_STATUS, &status);      /* read status             */
        if (rc == RMEGB_OK && (status & RMEGB_STATUS_INITIALIZED))   /* bit2 set?      */
            return RMEGB_OK;                                /* homed, ready to move    */
        if (rc != RMEGB_OK && rc != RMEGB_E_TIMEOUT &&      /* not just a missed ...   */
            rc != RMEGB_E_CRC)                              /* ... or garbled reply?   */
            return rc;                                      /* real error: give up     */
        port_sleep_ms(RMEGB_STATUS_POLL_MS);                /* poll period             */
    }
    return RMEGB_E_NOT_READY;                               /* did not finish in time  */
}

int rmegb_save_parameters(rmegb *g)
{
    int rc;                                                 /* result                  */

    if (!g || !g->port) return RMEGB_E_ARG;                 /* not open                */
    rc = write_coil(g, RMEGB_COIL_SAVE, 0);                 /* coil low: arm the edge  */
    if (rc != RMEGB_OK) return rc;                          /* not accepted            */
    rc = write_coil(g, RMEGB_COIL_SAVE, 1);                 /* coil high: save now     */
    if (rc != RMEGB_OK) return rc;                          /* not accepted            */
    port_sleep_ms(RMEGB_SAVE_SETTLE_MS);                    /* let the memory write end*/
    return RMEGB_OK;                                        /* saved                   */
}
