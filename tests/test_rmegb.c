/*
 * test_rmegb.c - offline test of rmegb.c against a fake port.
 * The fake port replaces rmegb_serial_*.c: it records what is sent and
 * replays scripted replies. Frames are the examples from the manual,
 * pages 15-19.
 */
#include "rmegb.h"                      /* code under test                            */
#include "../src/rmegb_port.h"          /* functions the fake port must provide       */

#include <stdio.h>                      /* printf, sscanf                             */
#include <string.h>                     /* memcpy, memcmp                             */

static int fails = 0, checks = 0;       /* test counters                              */
#define CHECK(c) do { checks++; if (!(c)) { fails++;  /* count, report failures */ \
    printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

/* "01 05 00 11" -> bytes; returns the count */
static size_t hex(const char *s, uint8_t *out)
{
    size_t n = 0; unsigned v; int k;                        /* count, value, chars used*/
    while (sscanf(s, " %2x%n", &v, &k) == 1) {              /* next hex byte           */
        out[n++] = (uint8_t)v; s += k;                      /* store it, move on       */
    }
    return n;                                               /* bytes parsed            */
}

/* ---- fake port ---------------------------------------------------------- */
static uint32_t clock_ms;                                   /* simulated time          */
static uint32_t slept_ms;                                   /* total time slept        */
static uint8_t  replies[32][32];                            /* scripted replies        */
static size_t   reply_len[32];                              /* their lengths           */
static int      nreplies, next_reply;                       /* count, next to send     */
static uint8_t  rx[32];                                     /* reply being received    */
static size_t   rx_len, rx_pos;                             /* its length, read pos    */
static uint8_t  sent[32][32];                               /* recorded requests       */
static size_t   sent_len[32];                               /* their lengths           */
static int      nsent;                                      /* request count           */
static int      fail_write, fail_read;                      /* simulate port errors    */

static void reset(void)                                     /* start a fresh test      */
{
    clock_ms = 1000; slept_ms = 0;                          /* time                    */
    nreplies = next_reply = nsent = 0;                      /* scripts and records     */
    rx_len = rx_pos = 0;                                    /* receive buffer          */
    fail_write = fail_read = 0;                             /* port works              */
}
static void reply(const char *h)                            /* script one reply        */
{
    reply_len[nreplies] = hex(h, replies[nreplies]);        /* store it                */
    nreplies++;                                             /* one more                */
}
static void reply_raw(const uint8_t *b, size_t n)           /* script raw reply bytes  */
{
    memcpy(replies[nreplies], b, n);                        /* store them              */
    reply_len[nreplies++] = n;                              /* and their length        */
}
static void reply_crc(const char *h)                        /* script reply, add CRC   */
{
    uint8_t b[32]; size_t n = hex(h, b);                    /* body bytes              */
    uint16_t c = 0xFFFFu; size_t k; int j;                  /* CRC-16/MODBUS ...       */
    for (k = 0; k < n; k++) {                               /* ... computed here       */
        c ^= b[k];                                          /* independently of the    */
        for (j = 0; j < 8; j++)                             /* library's own crc16()   */
            c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
    }
    b[n++] = (uint8_t)(c & 0xFFu); b[n++] = (uint8_t)(c >> 8);  /* append, low first  */
    reply_raw(b, n);                                        /* queue it                */
}
static int sent_is(int i, const char *h)                    /* was request i == h?     */
{
    uint8_t b[32]; size_t n = hex(h, b);                    /* expected bytes          */
    return i < nsent && sent_len[i] == n && memcmp(sent[i], b, n) == 0;  /* compare    */
}

int port_open(void **h, const char *name, uint32_t baud, int *e)   /* always works    */
{ (void)name; (void)baud; (void)e; *h = (void *)1; return 0; }
void port_close(void *h) { (void)h; }                       /* nothing to close        */
int port_write(void *h, const uint8_t *d, size_t n)         /* record + queue reply    */
{
    (void)h;                                                /* unused                  */
    if (fail_write) return -1;                              /* simulated port failure  */
    if (nsent >= 32) nsent = 0;                             /* fuzzing: reuse slots    */
    memcpy(sent[nsent], d, n); sent_len[nsent++] = n;       /* record the request      */
    rx_len = rx_pos = 0;                                    /* no reply yet            */
    if (next_reply < nreplies) {                            /* scripted reply left?    */
        memcpy(rx, replies[next_reply], reply_len[next_reply]);  /* "receive" it      */
        rx_len = reply_len[next_reply++];                   /* its length              */
    }
    clock_ms += 1;                                          /* sending takes 1 ms      */
    return 0;                                               /* success                 */
}
int port_read(void *h, uint8_t *buf, size_t max, uint32_t timeout_ms)  /* deliver reply */
{
    size_t n = rx_len - rx_pos;                             /* bytes left              */
    (void)h;                                                /* unused                  */
    if (fail_read) return -1;                               /* simulated port failure  */
    if (n == 0) { clock_ms += timeout_ms; return 0; }       /* none: time out          */
    if (n > 3) n = 3;                                       /* arrive in 3-byte pieces */
    if (n > max) n = max;                                   /* not more than asked     */
    memcpy(buf, rx + rx_pos, n); rx_pos += n;               /* hand them over          */
    clock_ms += 1;                                          /* 1 ms per piece          */
    return (int)n;                                          /* bytes delivered         */
}
void port_flush_rx(void *h) { (void)h; }                    /* buffer is per request   */
uint32_t port_millis(void) { return clock_ms; }             /* simulated clock         */
void port_sleep_ms(uint32_t ms) { clock_ms += ms; slept_ms += ms; }  /* advance clock  */

/* ---- tests -------------------------------------------------------------- */
int main(void)
{
    rmegb g;                                                /* connection              */
    uint32_t v = 0;                                         /* read value              */
    int i;                                                  /* loop counter            */

    /* initialize: coil 0 then 1 (p.15), then status polling */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 05 00 11 00 00 9D CF");                       /* echo of coil = 0        */
    reply("01 05 00 11 FF 00 DC 3F");                       /* echo of coil = 1        */
    reply("01 03 04 00 05 00 00 EA 32");                    /* stale: still initialized*/
    reply("01 03 04 00 00 00 00 FA 33");                    /* bit2 cleared: homing    */
    reply("");                                              /* no answer while busy    */
    reply("01 03 04 00 04 00 00 BB F2");                    /* bit2 set: done          */
    CHECK(rmegb_initialize(&g, 5000) == RMEGB_OK);          /* succeeds                */
    CHECK(sent_is(0, "01 05 00 11 00 00 9D CF"));           /* manual frame, coil 0    */
    CHECK(sent_is(1, "01 05 00 11 FF 00 DC 3F"));           /* manual frame, coil 1    */
    CHECK(sent_is(2, "01 03 10 08 00 02 41 09"));           /* then status reads       */
    CHECK(nsent == 6);                                      /* stopped at "done"       */

    /* initialize that never finishes */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 05 00 11 00 00 9D CF");                       /* echo coil 0             */
    reply("01 05 00 11 FF 00 DC 3F");                       /* echo coil 1             */
    for (i = 0; i < 30; i++) reply("01 03 04 00 00 00 00 FA 33");  /* never bit2      */
    CHECK(rmegb_initialize(&g, 300) == RMEGB_E_NOT_READY);  /* gives up after 300 ms   */

    /* save parameters: coil 0 then 1 (p.19), then a pause */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 05 00 09 00 00 1D C8");                       /* echo coil 0             */
    reply("01 05 00 09 FF 00 5C 38");                       /* echo coil 1             */
    CHECK(rmegb_save_parameters(&g) == RMEGB_OK);           /* succeeds                */
    CHECK(sent_is(0, "01 05 00 09 00 00 1D C8"));           /* manual frame, coil 0    */
    CHECK(sent_is(1, "01 05 00 09 FF 00 5C 38"));           /* manual frame, coil 1    */
    CHECK(slept_ms >= 100);                                 /* waited for the write    */

    /* force and position (p.16-17) */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 10 10 06 00 02 A5 09");                       /* ack force               */
    reply("01 10 10 04 00 02 04 C9");                       /* ack position            */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 50) == RMEGB_OK);       /* 50 % force       */
    CHECK(rmegb_write(&g, RMEGB_REG_POSITION, 75) == RMEGB_OK);    /* 75 % opening     */
    CHECK(sent_is(0, "01 10 10 06 00 02 04 00 32 00 00 1F 8A"));   /* manual frame     */
    CHECK(sent_is(1, "01 10 10 04 00 02 04 00 4B 00 00 4F 8A"));   /* manual frame     */

    /* out-of-range values are clamped to the nearest bound */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    for (i = 0; i < 3; i++) reply("01 10 10 06 00 02 A5 09");      /* 3 force acks     */
    for (i = 0; i < 2; i++) reply("01 10 10 04 00 02 04 C9");      /* 2 position acks  */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 10) == RMEGB_OK);       /* below 25 %       */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 150) == RMEGB_OK);      /* above 100 %      */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, -5) == RMEGB_OK);   /* negative   */
    CHECK(rmegb_write(&g, RMEGB_REG_POSITION, 101) == RMEGB_OK);   /* above 100 %      */
    CHECK(rmegb_write(&g, RMEGB_REG_POSITION, -5) == RMEGB_OK);/* negative   */
    CHECK(sent_is(0, "01 10 10 06 00 02 04 00 19 00 00 6F 82"));   /* sent as 25 %     */
    CHECK(sent_is(1, "01 10 10 06 00 02 04 00 64 00 00 FF 9A"));   /* sent as 100 %    */
    CHECK(sent_is(2, "01 10 10 06 00 02 04 00 19 00 00 6F 82"));   /* sent as 25 %     */
    CHECK(sent_is(3, "01 10 10 04 00 02 04 00 64 00 00 7E 43"));   /* sent as 100 %    */
    CHECK(sent_is(4, "01 10 10 04 00 02 04 00 00 00 00 3F 9C"));   /* sent as 0 %      */

    /* status and position reads (p.18) */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 03 04 00 07 00 00 4B F2");                    /* status 7                */
    reply("01 03 04 01 EC 00 00 3A 3A");                    /* position 492            */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_OK && v == 7);       /* decoded */
    CHECK(rmegb_read(&g, RMEGB_REG_ACTUAL_POS, &v) == RMEGB_OK && v == 492); /* 4.92 mm */
    CHECK(sent_is(0, "01 03 10 08 00 02 41 09"));           /* manual frame            */
    CHECK(sent_is(1, "01 03 10 10 00 02 C1 0E"));           /* manual frame            */

    /* baud rate and station number (p.19) */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 10 1B 58 00 02 C6 FF");                       /* ack baud                */
    reply("01 10 1B 5A 00 02 67 3F");                       /* ack station             */
    CHECK(rmegb_write(&g, RMEGB_REG_BAUD, 115200) == RMEGB_OK);    /* write baud       */
    CHECK(rmegb_write(&g, RMEGB_REG_STATION, 3) == RMEGB_OK);      /* write station    */
    CHECK(sent_is(0, "01 10 1B 58 00 02 04 C2 00 00 01 B5 BD"));   /* manual frame     */
    CHECK(sent_is(1, "01 10 1B 5A 00 02 04 00 03 00 00 38 1C"));   /* manual frame     */

    /* error handling */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_TIMEOUT);  /* no reply       */
    reply("01 03 04 00 07 00 00 4B F3");                    /* wrong CRC               */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_CRC);      /* detected       */
    reply("01 03 04 00 07");                                /* cut-off reply           */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_TIMEOUT);  /* detected       */
    reply("01 83 02 C0 F1");                                /* exception 2             */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_EXCEPTION &&
          g.last_exception == 2);                           /* code kept               */
    reply("01 10 10 06 00 02 A5 09");                       /* ack for another register*/
    CHECK(rmegb_write(&g, RMEGB_REG_POSITION, 50) == RMEGB_E_FRAME); /* detected       */

    /* broadcast: no reply awaited, turnaround pause, reads refused */
    reset(); rmegb_open(&g, "x", 115200, 0);                /* station 0               */
    CHECK(rmegb_write(&g, RMEGB_REG_POSITION, 50) == RMEGB_OK);    /* sent             */
    CHECK(slept_ms >= 100);                                 /* paused afterwards       */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_ARG);    /* nobody answers   */

    /* inter-frame gap */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 03 04 00 07 00 00 4B F2");                    /* two replies             */
    reply("01 03 04 00 07 00 00 4B F2");
    rmegb_read(&g, RMEGB_REG_STATUS, &v);                   /* first request           */
    CHECK(slept_ms == 0);                                   /* no wait before first    */
    rmegb_read(&g, RMEGB_REG_STATUS, &v);                   /* immediately again       */
    CHECK(slept_ms == 3);                                   /* 2 ms gap + 1 ms rounding*/
    rmegb_open(&g, "x", 9600, 1);                           /* slower link             */
    CHECK(g.gap_ms == 6);                                   /* longer gap              */

    /* ---- argument checks ---------------------------------------------- */
    CHECK(rmegb_open(NULL, "x", 115200, 1) == RMEGB_E_ARG); /* no struct               */
    CHECK(rmegb_open(&g, NULL, 115200, 1) == RMEGB_E_ARG);  /* no port name            */
    reset(); rmegb_open(&g, "x", 0, 1);                     /* baud 0 = default        */
    CHECK(g.gap_ms == 2 && g.timeout_ms == RMEGB_TIMEOUT_MS);   /* 115200 timing       */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, NULL) == RMEGB_E_ARG);  /* nowhere to store */
    CHECK(rmegb_write(NULL, RMEGB_REG_FORCE, 50) == RMEGB_E_ARG);  /* no struct        */
    rmegb_close(&g);                                        /* close ...               */
    rmegb_close(&g);                                        /* ... twice is harmless   */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 50) == RMEGB_E_ARG);    /* closed           */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_ARG);    /* closed           */
    CHECK(rmegb_initialize(&g, 100) == RMEGB_E_ARG);        /* closed                  */
    CHECK(rmegb_save_parameters(&g) == RMEGB_E_ARG);        /* closed                  */
    CHECK(nsent == 0);                                      /* nothing was sent        */

    /* ---- clamping edges --------------------------------------------------- */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    for (i = 0; i < 6; i++) reply("01 10 10 06 00 02 A5 09");      /* force acks       */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 25) == RMEGB_OK);       /* exactly 25       */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, 100) == RMEGB_OK);      /* exactly 100      */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, INT32_MIN) == RMEGB_OK);/* most negative    */
    CHECK(rmegb_write(&g, RMEGB_REG_FORCE, INT32_MAX) == RMEGB_OK);/* most positive    */
    CHECK(sent_is(0, "01 10 10 06 00 02 04 00 19 00 00 6F 82"));   /* 25 unchanged     */
    CHECK(sent_is(1, "01 10 10 06 00 02 04 00 64 00 00 FF 9A"));   /* 100 unchanged    */
    CHECK(sent_is(2, "01 10 10 06 00 02 04 00 19 00 00 6F 82"));   /* -> 25            */
    CHECK(sent_is(3, "01 10 10 06 00 02 04 00 64 00 00 FF 9A"));   /* -> 100           */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* other registers are ... */
    reply_crc("01 10 1B 5A 00 02");                         /* ... not clamped         */
    CHECK(rmegb_write(&g, RMEGB_REG_STATION, 300) == RMEGB_OK);    /* passed through   */
    CHECK(sent[0][7] == 0x01 && sent[0][8] == 0x2C);        /* 300 = 0x012C as sent    */

    /* ---- 32-bit decoding with a non-zero high word ------------------------ */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply_crc("01 03 04 C2 00 00 01");                      /* 115200 = 0x0001C200     */
    CHECK(rmegb_read(&g, RMEGB_REG_BAUD, &v) == RMEGB_OK && v == 115200u);  /* decoded */

    /* ---- malformed replies ------------------------------------------------ */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply_crc("01 03 02 00 07 00 00");                      /* byte count 2, not 4     */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_FRAME);  /* rejected         */
    reply_crc("01 04 04 00 07 00 00");                      /* wrong function code     */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_FRAME);  /* rejected         */
    reply_crc("02 03 04 00 07 00 00");                      /* other station           */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_FRAME);  /* rejected         */
    reply_crc("01 03 04 00 07 00 00 55 55");                /* extra bytes after CRC?  */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_CRC);    /* misaligned: bad  */

    /* ---- coil errors ------------------------------------------------------ */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 05 00 11 FF 00 DC 3F");                       /* echo says ON, sent OFF  */
    CHECK(rmegb_initialize(&g, 1000) == RMEGB_E_FRAME);     /* mismatch detected       */
    CHECK(nsent == 1);                                      /* stopped before coil = 1 */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply_crc("01 85 02");                                  /* exception to FC05       */
    CHECK(rmegb_initialize(&g, 1000) == RMEGB_E_EXCEPTION && g.last_exception == 2);
    CHECK(nsent == 1);                                      /* no polling afterwards   */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    reply("01 05 00 09 00 00 1D C8");                       /* coil 0 ok ...           */
    CHECK(rmegb_save_parameters(&g) == RMEGB_E_TIMEOUT);    /* ... coil 1 unanswered   */

    /* ---- port failures ---------------------------------------------------- */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* fresh connection        */
    fail_write = 1;                                         /* adapter unplugged       */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_IO);     /* reported         */
    fail_write = 0; fail_read = 1;                          /* now reads fail          */
    reply("01 03 04 00 07 00 00 4B F2");
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_IO);     /* reported         */

    /* ---- initialization edge cases ---------------------------------------- */
    reset(); rmegb_open(&g, "x", 115200, 1);                /* bit2 never clears:      */
    reply("01 05 00 11 00 00 9D CF"); reply("01 05 00 11 FF 00 DC 3F");
    for (i = 0; i < 30; i++) reply("01 03 04 00 05 00 00 EA 32");  /* always 5        */
    {
        uint32_t t0 = clock_ms;                             /* start time              */
        CHECK(rmegb_initialize(&g, 5000) == RMEGB_OK);      /* accepted after window   */
        CHECK(clock_ms - t0 >= RMEGB_INIT_START_WINDOW_MS); /* but not before it       */
    }
    reset(); rmegb_open(&g, "x", 115200, 1);                /* CRC error while homing  */
    reply("01 05 00 11 00 00 9D CF"); reply("01 05 00 11 FF 00 DC 3F");
    reply("01 03 04 00 00 00 00 FA 33");                    /* homing                  */
    reply("01 03 04 00 00 00 00 FA 34");                    /* corrupted read          */
    reply("01 03 04 00 04 00 00 BB F2");                    /* homed                   */
    CHECK(rmegb_initialize(&g, 5000) == RMEGB_OK);          /* noise tolerated         */
    reset(); rmegb_open(&g, "x", 115200, 0);                /* broadcast init          */
    CHECK(rmegb_initialize(&g, 5000) == RMEGB_OK);          /* sent, no polling        */
    CHECK(nsent == 2);                                      /* just the two coil writes*/

    /* ---- clock wrap-around (port_millis passes 0xFFFFFFFF -> 0) ----------- */
    reset(); clock_ms = 0xFFFFFFF0u;                        /* 16 ms before the wrap   */
    rmegb_open(&g, "x", 115200, 1);                         /* fresh connection        */
    reply("01 03 04 00 07 00 00 4B F2"); reply("01 03 04 00 07 00 00 4B F2");
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_OK);   /* before the wrap      */
    CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_OK);   /* gap still 2+1 ms     */
    CHECK(slept_ms == 3);                                   /* not ~4 billion ms       */
    {
        uint32_t t0 = clock_ms;                             /* start time              */
        CHECK(rmegb_read(&g, RMEGB_REG_STATUS, &v) == RMEGB_E_TIMEOUT);  /* across wrap */
        CHECK(clock_ms - t0 < 2 * RMEGB_TIMEOUT_MS);        /* timed out on time       */
    }

    /* ---- fuzz: random replies must never crash or be accepted wrongly ----- */
    {
        uint32_t seed = 12345u;                             /* fixed: reproducible     */
        long accepted = 0, n;                               /* replies taken as valid  */
        for (n = 0; n < 200000; n++) {                      /* many random replies     */
            uint8_t b[24]; size_t len, k;                   /* reply bytes, length     */
            int which, rc;                                  /* call to test, result    */
            seed = seed * 1103515245u + 12345u;             /* simple LCG              */
            len = (seed >> 16) % 20u;                       /* 0..19 bytes             */
            for (k = 0; k < len; k++) {                     /* random content, but ... */
                seed = seed * 1103515245u + 12345u;
                b[k] = (uint8_t)(seed >> 16);
            }
            if (len > 2 && (n & 1)) {                       /* ... half the time keep  */
                b[0] = 0x01;                                /* station and a valid     */
                b[1] = (n & 2) ? 0x03 : 0x10;               /* function code so the    */
            }                                               /* deeper checks run       */
            reset(); rmegb_open(&g, "x", 115200, 1);        /* fresh connection        */
            if (len >= 3 && (n % 3 == 0)) {                 /* every 3rd: valid CRC    */
                uint16_t c = 0xFFFFu; int j;
                for (k = 0; k + 2 < len; k++) {
                    c ^= b[k];
                    for (j = 0; j < 8; j++)
                        c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
                }
                b[len - 2] = (uint8_t)(c & 0xFFu); b[len - 1] = (uint8_t)(c >> 8);
            }
            reply_raw(b, len);                              /* queue the random reply  */
            which = (int)(n % 2);                           /* alternate read / write  */
            rc = which ? rmegb_read(&g, RMEGB_REG_STATUS, &v)
                       : rmegb_write(&g, RMEGB_REG_POSITION, 50);
            CHECK(rc <= 0 && rc >= RMEGB_E_NOT_READY);      /* a known code            */
            if (rc == RMEGB_OK) {                           /* accepted: must be right */
                accepted++;
                if (which) CHECK(b[0] == 1 && b[1] == 3 && b[2] == 4 && len >= 9);
                else       CHECK(len >= 8 && memcmp(b, sent[0], 6) == 0);
            }
        }
        printf("fuzz: 200000 random replies, %ld accepted (all checked valid)\n", accepted);
    }

    printf("%d checks, %d failed\n", checks, fails);        /* summary                 */
    return fails != 0;                                      /* exit code for ctest     */
}
