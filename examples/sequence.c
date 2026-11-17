/*
 * sequence.c - initialize (home) the gripper, then run through the steps
 *              in the table below: fully open, fully close, then the same
 *              open/close cycle at 50 % and at 100 % force and opening.
 *
 *   sequence COM5            (Windows)
 *   sequence /dev/ttyUSB0    (Linux)
 */
#include "rmegb.h"                      /* gripper API                                */

#include <stdio.h>                      /* printf, fprintf                            */

/* Which end of the stroke is "open". The manual does not say whether 0 % is
   open or closed. If the gripper closes when it should open, swap these two. */
static const int32_t open_pct   = 100;  /* position written for "fully open"          */
static const int32_t closed_pct = 0;    /* position written for "fully closed"        */

typedef struct {
    const char *name;                   /* printed before the step                    */
    int32_t     force;                  /* force, % of maximum (25..100)              */
    int32_t     position;               /* target opening, % of stroke (0..100)       */
} step;

/* Wait until the motion started by the last position write has finished.
   Right after the write, bit0 (in place) may still show the previous motion
   as finished, so first wait briefly for it to clear (motion started), then
   for it to set again. If the gripper is already at the target, bit0 may
   never clear; the first loop then just runs out. Each read takes a few ms
   (more if the FTDI latency timer is not set to 1 ms).
   A single lost (timeout) or garbled (CRC) status reply is skipped and the
   next read tried, as rmegb_initialize() does: reading the status twice is
   harmless, and without this one lost reply aborts the whole sequence. */
static int wait_motion(rmegb *g, uint32_t *status)
{
    int rc = RMEGB_E_TIMEOUT;                               /* result of last read     */
    int i;                                                  /* loop counter            */

    for (i = 0; i < 20; i++) {                              /* up to 20 reads:         */
        rc = rmegb_read(g, RMEGB_REG_STATUS, status);       /* read status             */
        if (rc == RMEGB_E_TIMEOUT || rc == RMEGB_E_CRC) continue;  /* lost: read again */
        if (rc != RMEGB_OK) return rc;                      /* real problem: stop      */
        if (!(*status & RMEGB_STATUS_IN_PLACE)) break;      /* moving now              */
    }
    for (i = 0; i < 1000; i++) {                            /* up to 1000 reads:       */
        rc = rmegb_read(g, RMEGB_REG_STATUS, status);       /* read status             */
        if (rc == RMEGB_E_TIMEOUT || rc == RMEGB_E_CRC) continue;  /* lost: read again */
        if (rc != RMEGB_OK) return rc;                      /* real problem: stop      */
        if (*status & RMEGB_STATUS_IN_PLACE) return RMEGB_OK;  /* arrived              */
    }
    return rc == RMEGB_OK ? RMEGB_E_TIMEOUT : rc;           /* never arrived           */
}

int main(int argc, char **argv)
{
    const step steps[] = {                                  /* the sequence            */
        { "fully open                 (force 100 %)", 100, open_pct   },
        { "fully close                (force 100 %)", 100, closed_pct },
        { "open to  50 %              (force  50 %)",  50, 50         },
        { "fully close                (force  50 %)",  50, closed_pct },
        { "open to 100 %              (force 100 %)", 100, 100        },
        { "fully close                (force 100 %)", 100, closed_pct },
    };
    const int nsteps = (int)(sizeof steps / sizeof steps[0]);  /* number of steps      */

    rmegb g;                                                /* connection              */
    uint32_t status = 0;                                    /* status register value   */
    uint32_t pos = 0;                                       /* actual position, 0.01 mm*/
    int rc;                                                 /* result of each call     */
    int s;                                                  /* step index              */

    if (argc < 2) {                                         /* no port given?          */
        fprintf(stderr, "usage: %s PORT\n", argv[0]);       /* show usage              */
        return 2;                                           /* exit                    */
    }

    rc = rmegb_open(&g, argv[1], 115200, 1);                /* 115200 baud, station 1  */
    if (rc != RMEGB_OK) {                                   /* could not open?         */
        fprintf(stderr, "cannot open %s (OS error %d)\n", argv[1], g.os_error);  /* why*/
        return 2;                                           /* exit                    */
    }

    printf("initializing (homing)...\n");                   /* tell the user           */
    rc = rmegb_initialize(&g, 10000);                       /* home, wait up to 10 s   */
    if (rc == RMEGB_OK) printf("initialized\n");            /* homed                   */

    for (s = 0; rc == RMEGB_OK && s < nsteps; s++) {        /* run every step          */
        printf("step %d: %s ... ", s + 1, steps[s].name);   /* announce the step       */
        fflush(stdout);                                     /* show it before waiting  */
        rc = rmegb_write(&g, RMEGB_REG_FORCE, steps[s].force);  /* force first: it is  */
        if (rc == RMEGB_OK)                                 /* applied by the next     */
            rc = rmegb_write(&g, RMEGB_REG_POSITION, steps[s].position);  /* position  */
        if (rc == RMEGB_OK) rc = wait_motion(&g, &status);  /* wait until in place     */
        if (rc == RMEGB_OK) rc = rmegb_read(&g, RMEGB_REG_ACTUAL_POS, &pos);  /* where */
        if (rc == RMEGB_OK)                                 /* step done:              */
            printf("at %.2f mm%s\n", (double)pos / 100.0,   /* position in mm          */
                   (status & RMEGB_STATUS_FORCE_REACHED) ? ", force reached (part held)" : "");
    }

    if (rc == RMEGB_OK) {                                   /* everything worked       */
        printf("sequence complete\n");                      /* say so                  */
    } else if (rc == RMEGB_E_EXCEPTION) {                   /* gripper refused:        */
        printf("\nerror: gripper refused request, exception %d\n", g.last_exception);
    } else {                                                /* anything else:          */
        printf("\nerror %d (see return codes in rmegb.h)\n", rc);  /* report the code  */
    }

    rmegb_close(&g);                                        /* release the port        */
    return rc == RMEGB_OK ? 0 : 1;                          /* exit code for scripts   */
}
