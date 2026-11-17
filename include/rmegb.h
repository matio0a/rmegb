/*
 * rmegb.h - Modbus RTU access to RobustMotion RM-EGB grippers
 *           (user manual EGBPUM2409, section 4, pages 13-19).
 *
 *   rmegb_write()            write one holding register (FC10)
 *   rmegb_read()             read one holding register (FC03)
 *   rmegb_initialize()       home the gripper and wait until it is ready
 *   rmegb_save_parameters()  store station number and baud rate
 *
 * Works on Windows and Linux through a USB-RS485 adapter.
 */
#ifndef RMEGB_H                         /* include guard: skip if already included   */
#define RMEGB_H                         /* mark this header as included              */

#include <stdint.h>                     /* uint8_t, uint16_t, uint32_t               */

#ifdef __cplusplus                      /* when compiled as C++ ...                  */
extern "C" {                            /* ... keep C names so C++ code can link     */
#endif

/* ---- Modbus function codes --------------------------------------------- */
#define RMEGB_FC_READ_HOLDING    0x03u /* read holding registers                        */
#define RMEGB_FC_WRITE_COIL      0x05u /* write single coil                             */
#define RMEGB_FC_WRITE_MULTIPLE  0x10u /* write multiple registers                      */
#define RMEGB_FC_EXCEPTION       0x80u /* added to the function code in error replies   */

/* ---- Timings (ms) ------------------------------------------------------- */
/* Reply timeout and pause after a broadcast.
   Reply timeout: RS-485 here is half-duplex master/slave, so if the gripper
   does not answer (unpowered, wrong station, wrong baud, broken wire, noise)
   nothing arrives at all; without a timeout the program would wait forever.
   The gripper normally answers within a few ms; 100 ms leaves room for USB
   scheduling and the FTDI latency timer.
   Broadcast pause: after a broadcast (station 0) nobody answers, so there is
   no reply telling us the grippers have finished with the frame. The Modbus
   serial line spec asks the master to pause ("turnaround delay", typically
   100-200 ms) before the next request. Without it the next request can
   arrive while the grippers are still busy and be ignored, which shows up
   as RMEGB_E_TIMEOUT. */
#define RMEGB_TIMEOUT_MS 100u

/* Status polling period while waiting for initialization. Polling without a
   pause works too (each read already takes a few ms), but keeps the bus and
   the gripper's processor busy for no benefit: homing takes hundreds of ms. */
#define RMEGB_STATUS_POLL_MS 10u

/* Right after the init command, the status can still show "initialized"
   from the previous homing, because the gripper needs a moment to clear
   bit2. Without this window rmegb_initialize() could return while homing is
   only starting, and a move sent then would be refused or lost (the manual
   says not to command the gripper during initialization). We wait up to
   this long to see bit2 go to 0 before waiting for it to come back to 1. */
#define RMEGB_INIT_START_WINDOW_MS 200u

/* Saving writes the parameters to the gripper's non-volatile memory. The
   manual gives no duration. Such writes typically take up to tens of ms, and
   a device may not answer while it writes. This pause keeps the next request
   from timing out and gives the write time to finish before anyone powers
   the gripper off (power loss during the write can corrupt the settings). */
#define RMEGB_SAVE_SETTLE_MS 100u

/* ---- Coil addresses (manual p.13), written with FC05 ------------------- */
#define RMEGB_COIL_SAVE        0x0009u  /* save station number and baud rate         */
#define RMEGB_COIL_INIT        0x0011u  /* initialization: home the fingers          */

/* ---- Holding register addresses (manual p.13), 32-bit ------------------ */
#define RMEGB_REG_POSITION     0x1004u  /* write: target opening 0..100 %, moves now */
#define RMEGB_REG_FORCE        0x1006u  /* write: force 25..100 %, used by next move */
#define RMEGB_REG_STATUS       0x1008u  /* read: status bits below                   */
#define RMEGB_REG_ACTUAL_POS   0x1010u  /* read: actual position in 0.01 mm          */
#define RMEGB_REG_BAUD         0x1B58u  /* write: 9600/19200/38400/57600/115200      */
#define RMEGB_REG_STATION      0x1B5Au  /* write: station number 1..255              */

/* ---- Status register bits (manual p.18) -------------------------------- */
#define RMEGB_STATUS_IN_PLACE       0x1u  /* bit0: motion completed                  */
#define RMEGB_STATUS_FORCE_REACHED  0x2u  /* bit1: set force reached = part gripped  */
#define RMEGB_STATUS_INITIALIZED    0x4u  /* bit2: initialization complete           */

/* ---- Return codes ------------------------------------------------------ */
#define RMEGB_OK            0   /* success                                           */
#define RMEGB_E_ARG        -1   /* port not open or NULL pointer; nothing sent       */
#define RMEGB_E_IO         -2   /* serial port read/write failed (unplugged?)        */
#define RMEGB_E_TIMEOUT    -3   /* no complete reply within timeout_ms               */
#define RMEGB_E_CRC        -4   /* reply corrupted                                   */
#define RMEGB_E_FRAME      -5   /* reply from wrong station or for another request   */
#define RMEGB_E_EXCEPTION  -6   /* gripper refused the request; see last_exception   */
#define RMEGB_E_OPEN       -7   /* port could not be opened; OS code in os_error     */
#define RMEGB_E_NOT_READY  -8   /* initialization did not finish within the timeout  */

/* ---- Connection state --------------------------------------------------- */
typedef struct rmegb {
    void    *port;            /* OS handle of the serial port, owned by port layer   */
    uint8_t  slave;           /* station number of the gripper; 0 = broadcast        */
    uint32_t timeout_ms;      /* how long to wait for a reply (default 100 ms)       */
    uint32_t gap_ms;          /* bus silence required between frames (from baud)     */
    uint32_t last_ms;         /* time the bus was last active, for the gap           */
    int      last_exception;  /* Modbus exception code after RMEGB_E_EXCEPTION       */
    int      os_error;        /* errno / GetLastError() after RMEGB_E_OPEN           */
} rmegb;

/* Open "COM5" / "/dev/ttyUSB0" at baud (gripper default 115200), station slave. */
int  rmegb_open(rmegb *g, const char *port, uint32_t baud, uint8_t slave);

/* Close the port. */
void rmegb_close(rmegb *g);

/* Write a signed 32-bit value to a holding register. Force is clamped to 25..100 %
   and position to 0..100 % (out-of-range values go to the nearest bound). */
int  rmegb_write(rmegb *g, uint16_t address, int32_t value);

/* Read a 32-bit holding register into *value. */
int  rmegb_read(rmegb *g, uint16_t address, uint32_t *value);

/* Home the gripper (init coil 0 then 1) and wait up to timeout_ms until the
   status says initialized. Not needed after power-on: the gripper homes by
   itself then (manual p.8, green LED). Use it to re-home on demand. */
int  rmegb_initialize(rmegb *g, uint32_t timeout_ms);

/* Store station number / baud rate written earlier (save coil 0 then 1). */
int  rmegb_save_parameters(rmegb *g);

#ifdef __cplusplus                      /* close the extern "C" block ...            */
}                                       /* ... opened above for C++                  */
#endif

#endif /* RMEGB_H */                    /* end of include guard                      */
