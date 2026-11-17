/*
 * grip.c - make sure the gripper is homed, move to `opening` with `force`,
 *          report whether a part was gripped.
 *
 *   grip COM5            (Windows)
 *   grip /dev/ttyUSB0    (Linux)
 */
#include "rmegb.h"                      /* gripper API                                */

#include <stdio.h>                      /* printf, fprintf                            */

int main(int argc, char **argv)
{
    int32_t opening = 0;      /* target opening, % of stroke (0..100). The manual
                                 does not say whether 0 % is open or closed:
                                 check once on the real gripper.             */
    int32_t force   = 40;     /* gripping force, % of maximum (25..100)      */

    rmegb g;                                                /* connection              */
    uint32_t status = 0;                                    /* status register value   */
    uint32_t pos = 0;                                       /* actual position, 0.01 mm*/
    int rc;                                                 /* result of each call     */
    int r;                                                  /* result of one poll read */
    int arrived;                                            /* 1 once in place         */
    int i;                                                  /* loop counter            */

    if (argc < 2) {                                         /* no port given?          */
        fprintf(stderr, "usage: %s PORT\n", argv[0]);       /* show usage              */
        return 2;                                           /* exit                    */
    }

    rc = rmegb_open(&g, argv[1], 115200, 1);                /* 115200 baud, station 1  */
    if (rc != RMEGB_OK) {                                   /* could not open?         */
        fprintf(stderr, "cannot open %s (OS error %d)\n", argv[1], g.os_error);  /* why*/
        return 2;                                           /* exit                    */
    }

    /* The gripper homes by itself at power-on. Only home it here if it
       reports that it is not initialized (e.g. still homing, or after a fault). */
    rc = rmegb_read(&g, RMEGB_REG_STATUS, &status);         /* read the status         */
    if (rc == RMEGB_OK && !(status & RMEGB_STATUS_INITIALIZED))  /* not homed?         */
        rc = rmegb_initialize(&g, 5000);                    /* home, wait up to 5 s    */

    if (rc == RMEGB_OK) rc = rmegb_write(&g, RMEGB_REG_FORCE, force);       /* force first:   */
    if (rc == RMEGB_OK) rc = rmegb_write(&g, RMEGB_REG_POSITION, opening);  /* used by this move */

    /* Wait for the motion to finish (bit0). Right after the position write,
       bit0 may still show the previous motion as finished. So first wait
       briefly for it to clear (motion started), then for it to set again.
       If the gripper is already at the target it may never clear; the first
       loop then just runs out. Each read takes a few ms. A lost (timeout) or
       garbled (CRC) status reply is skipped and the next read tried. */
    for (i = 0; rc == RMEGB_OK && i < 20; i++) {            /* up to 20 reads:         */
        r = rmegb_read(&g, RMEGB_REG_STATUS, &status);      /* read status             */
        if (r == RMEGB_E_TIMEOUT || r == RMEGB_E_CRC) continue;  /* lost: read again   */
        rc = r;                                             /* keep real results       */
        if (rc == RMEGB_OK && !(status & RMEGB_STATUS_IN_PLACE)) break;  /* moving now */
    }
    arrived = 0;                                            /* not in place yet        */
    for (i = 0; rc == RMEGB_OK && i < 1000; i++) {          /* up to 1000 reads:       */
        r = rmegb_read(&g, RMEGB_REG_STATUS, &status);      /* read status             */
        if (r == RMEGB_E_TIMEOUT || r == RMEGB_E_CRC) continue;  /* lost: read again   */
        rc = r;                                             /* keep real results       */
        if (rc == RMEGB_OK && (status & RMEGB_STATUS_IN_PLACE)) { arrived = 1; break; }
    }
    if (rc == RMEGB_OK && !arrived) rc = RMEGB_E_TIMEOUT;   /* never got there         */
    if (rc == RMEGB_OK) rc = rmegb_read(&g, RMEGB_REG_ACTUAL_POS, &pos);  /* where now */

    if (rc == RMEGB_OK) {                                   /* all went well:          */
        printf("status %lu, position %.2f mm, part %s\n",   /* print result            */
               (unsigned long)status,                       /* raw status              */
               (double)pos / 100.0,                         /* 0.01 mm -> mm           */
               (status & RMEGB_STATUS_FORCE_REACHED) ? "gripped" : "not detected");
    } else if (rc == RMEGB_E_EXCEPTION) {                   /* gripper refused:        */
        fprintf(stderr, "gripper refused request, exception %d\n", g.last_exception);
    } else {                                                /* anything else:          */
        fprintf(stderr, "error %d\n", rc);                  /* see codes in rmegb.h    */
    }

    rmegb_close(&g);                                        /* release the port        */
    return rc == RMEGB_OK ? 0 : 1;                          /* exit code for scripts   */
}
