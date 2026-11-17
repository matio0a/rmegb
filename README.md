# rmegb — RM-EGB gripper driver

## 1. General description

`rmegb` is a small C99 driver for **RobustMotion RM-EGB** compact electric
grippers controlled over **Modbus RTU (RS-485)**. It runs on **Windows 10/11**
and **Linux** (macOS should also work) through a USB-RS485 adapter such as the
DSD TECH [SH-U11F](https://www.cncwiki.org/images/b/b6/DSD_Tech_SH-U11F_User_Guide.pdf), and can be used directly from C/C++ or built as a shared
library to be used from Python and other languages.

The register map and frame format follow the
[RM-EGB user manual](https://www.pololu.com/file/0J2112/RobustMotion%20RM-EGB%20gripper%20user%20manual.pdf)
(EGBPUM2409, section 4, pages 13–19). The driver handles the Modbus RTU
details: CRC, the inter-frame gap, reply timeouts, reply validation and the
gripper's word order for 32-bit values (low 16-bit word first).

**Wiring (SH-U11F, 2-wire RS-485)**

| Adapter | Gripper wire |
|---|---|
| A+ (TXD+) | Yellow (DSW+ / 485+) |
| B− (TXD−) | Green (DSW− / 485−) |
| GND | Blue (0 V) |

Gripper power: Brown +24 V, Blue 0 V. Gripper defaults: station 1, 115200 baud, 8N1.

**FTDI latency timer.** Set it to 1 ms, or every transaction takes 16–32 ms
instead of a few ms. Linux: set automatically when the port opens. Windows:
Device Manager → Ports → USB Serial Port → Properties → Port Settings →
Advanced → Latency Timer (msec) = 1.

**Quick start**

```c
#include "rmegb.h"

rmegb g;
int32_t opening = 0, force = 40;
uint32_t status;

rmegb_open(&g, "COM5", 115200, 1);            /* or "/dev/ttyUSB0" */
rmegb_initialize(&g, 5000);                   /* optional: it homes itself at power-on */
rmegb_write(&g, RMEGB_REG_FORCE, force);      /* applied by the next position write */
rmegb_write(&g, RMEGB_REG_POSITION, opening);
rmegb_read(&g, RMEGB_REG_STATUS, &status);    /* poll until RMEGB_STATUS_IN_PLACE */
rmegb_close(&g);
```

**Building the examples and tests (CMake ≥ 3.13)**

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release      # offline tests, no hardware needed
```

On Linux your user must be in the `dialout` group to open the port.

**Files**

```
include/rmegb.h            API, addresses, status bits, return codes, timings
src/rmegb.c                Modbus RTU framing and timing
src/rmegb_port.h           OS interface used by rmegb.c
src/rmegb_serial_posix.c   serial port for Linux / macOS
src/rmegb_serial_win32.c   serial port for Windows
examples/grip.c            make sure it is homed, move with force, report part
examples/sequence.c        home, open/close at 100 %, 50 % and 100 % force
tests/test_rmegb.c         offline tests (manual's example frames as vectors)
build_lib.sh               shared library for Linux / macOS
build_dll.bat              DLL for Windows (MSVC or MinGW-w64)
```

## 2. Function description

All functions return `RMEGB_OK` (0) or a negative error code, except
`rmegb_close()`. Calls are blocking and sequential: each one sends a frame and
waits for the reply (or the timeout) before returning. Not thread-safe: use
one `rmegb` per thread, or lock around calls.

### `int rmegb_open(rmegb *g, const char *port, uint32_t baud, uint8_t slave)`

Opens and configures the serial port (8N1, no flow control) and fills `g`.

- `port` — `"COM5"` on Windows (COM10+ work too), `"/dev/ttyUSB0"` or a
  `/dev/serial/by-id/...` link on Linux.
- `baud` — 9600, 19200, 38400, 57600 or 115200; `0` means 115200.
- `slave` — the gripper's station number (default 1). `0` is broadcast: all
  grippers act, none replies, reads are refused.
- Returns `RMEGB_E_OPEN` if the port cannot be opened; `g->os_error` then holds
  the OS error (Windows: 2 = no such port, 5 = in use; Linux: `errno`).

### `void rmegb_close(rmegb *g)`

Closes the port. Calling it twice is harmless.

### `int rmegb_write(rmegb *g, uint16_t address, int32_t value)`

Writes a 32-bit value to a holding register (Modbus function 10).

- `RMEGB_REG_FORCE` — force, 25–100 % of maximum. Takes effect with the
  **next** position write, so write force first.
- `RMEGB_REG_POSITION` — target opening, 0–100 % of stroke; the gripper starts
  moving at once. The manual does not say whether 0 % is open or closed.
- Force and position outside their range are clamped to the nearest bound
  (e.g. force 10 → 25, position −5 → 0, position 150 → 100).
- `RMEGB_REG_STATION`, `RMEGB_REG_BAUD` — sent unchanged; store them with
  `rmegb_save_parameters()`.
- Returns when the gripper has acknowledged the write, not when a motion has
  finished; poll the status register for that.

### `int rmegb_read(rmegb *g, uint16_t address, uint32_t *value)`

Reads a 32-bit holding register (Modbus function 03) into `*value`.

- `RMEGB_REG_STATUS` — status bits (below).
- `RMEGB_REG_ACTUAL_POS` — actual position in 0.01 mm (492 = 4.92 mm).

### `int rmegb_initialize(rmegb *g, uint32_t timeout_ms)`

Homes the gripper: writes the init coil 0 then 1 (rising edge, Modbus
function 05), then waits until the status reports "initialized" or
`timeout_ms` passes (`RMEGB_E_NOT_READY`). It first waits up to
`RMEGB_INIT_START_WINDOW_MS` for the old "initialized" bit to clear, so it
cannot return before homing has really started. A lost or garbled status
reply during homing is skipped. Not needed after power-on: the gripper homes
by itself then.

### `int rmegb_save_parameters(rmegb *g)`

Stores the station number and baud rate in the gripper's memory: writes the
save coil 0 then 1, then pauses `RMEGB_SAVE_SETTLE_MS`. Only needed after
changing those two settings.

### The `rmegb` structure

| Field | Meaning |
|---|---|
| `slave` | station number in use; may be changed between calls |
| `timeout_ms` | reply timeout, default `RMEGB_TIMEOUT_MS` (100 ms); may be changed |
| `last_exception` | Modbus exception code after `RMEGB_E_EXCEPTION` |
| `os_error` | OS error code after `RMEGB_E_OPEN` |
| `port`, `gap_ms`, `last_ms` | internal |

### Addresses

| Define | Address | Access |
|---|---|---|
| `RMEGB_REG_POSITION` | 0x1004 | write, 0–100 % |
| `RMEGB_REG_FORCE` | 0x1006 | write, 25–100 % |
| `RMEGB_REG_STATUS` | 0x1008 | read |
| `RMEGB_REG_ACTUAL_POS` | 0x1010 | read, 0.01 mm |
| `RMEGB_REG_BAUD` | 0x1B58 | write |
| `RMEGB_REG_STATION` | 0x1B5A | write, 1–255 |
| `RMEGB_COIL_SAVE` | 0x0009 | used by `rmegb_save_parameters()` |
| `RMEGB_COIL_INIT` | 0x0011 | used by `rmegb_initialize()` |

### Status bits

| Define | Bit | Meaning |
|---|---|---|
| `RMEGB_STATUS_IN_PLACE` | 0 | motion completed |
| `RMEGB_STATUS_FORCE_REACHED` | 1 | set force reached — a part is held |
| `RMEGB_STATUS_INITIALIZED` | 2 | initialization (homing) complete |

### Return codes

| Define | Value | Meaning |
|---|---|---|
| `RMEGB_OK` | 0 | success |
| `RMEGB_E_ARG` | −1 | port not open or NULL pointer; nothing sent |
| `RMEGB_E_IO` | −2 | serial port read/write failed (adapter unplugged?) |
| `RMEGB_E_TIMEOUT` | −3 | no complete reply within `timeout_ms` |
| `RMEGB_E_CRC` | −4 | reply corrupted |
| `RMEGB_E_FRAME` | −5 | reply from another station or for another request |
| `RMEGB_E_EXCEPTION` | −6 | gripper refused the request; code in `last_exception` |
| `RMEGB_E_OPEN` | −7 | port could not be opened; code in `os_error` |
| `RMEGB_E_NOT_READY` | −8 | initialization did not finish in time |

### Timings

| Define | Value | Purpose |
|---|---|---|
| `RMEGB_TIMEOUT_MS` | 100 ms | reply timeout; pause after broadcasts |
| `RMEGB_STATUS_POLL_MS` | 10 ms | status poll period while homing |
| `RMEGB_INIT_START_WINDOW_MS` | 200 ms | time allowed for "initialized" to clear |
| `RMEGB_SAVE_SETTLE_MS` | 100 ms | pause after saving parameters |

Why each delay is needed, and what happens without it, is explained in the
comments in `rmegb.h`, `rmegb.c` and the two serial port files.

## 3. How to build a .dll

To use the driver from Python or another language, build it as a shared
library. Only the six `rmegb_*` functions are exported; the internal
`port_*` functions stay hidden.

**Windows** — run from the project folder:

```bat
build_dll.bat
```

It uses MSVC if `cl` is on the PATH (start an *x64 Native Tools Command
Prompt for VS*), otherwise MinGW-w64 `gcc`, and writes `lib\rmegb.dll`. With
MSVC an import library `lib\rmegb.lib` is written too. The script defines
`RMEGB_DLL`, which marks the API functions `__declspec(dllexport)` in
`rmegb.h`; without it an MSVC DLL exports nothing. The MinGW build is linked
with `-static-libgcc`, so it needs no MinGW runtime on the target PC.

**Linux / macOS:**

```sh
./build_lib.sh
```

writes `lib/librmegb.so` (macOS: `lib/librmegb.dylib`), built with
`-fvisibility=hidden` so only the API is exported. No extra libraries are
needed.

**Check the exports** — both scripts list them after building; by hand:
`dumpbin /exports lib\rmegb.dll` (MSVC), `objdump -p lib\rmegb.dll` (MinGW),
`nm -D lib/librmegb.so` (Linux).

**Bitness** — build with the same bitness as the program that loads the
library: a 64-bit Python needs a 64-bit DLL (x64 prompt / 64-bit MinGW-w64).

**Using it from Python (ctypes).** Python must declare `struct rmegb` with the
same layout as `rmegb.h` (32 bytes on 64-bit systems):

```python
import ctypes as C

class Rmegb(C.Structure):
    _fields_ = [("port", C.c_void_p), ("slave", C.c_uint8), ("timeout_ms", C.c_uint32),
                ("gap_ms", C.c_uint32), ("last_ms", C.c_uint32),
                ("last_exception", C.c_int), ("os_error", C.c_int)]

lib = C.CDLL("lib/rmegb.dll")             # Linux: "lib/librmegb.so"
lib.rmegb_open.argtypes  = [C.POINTER(Rmegb), C.c_char_p, C.c_uint32, C.c_uint8]
lib.rmegb_close.argtypes = [C.POINTER(Rmegb)]
lib.rmegb_close.restype  = None
lib.rmegb_write.argtypes = [C.POINTER(Rmegb), C.c_uint16, C.c_int32]
lib.rmegb_read.argtypes  = [C.POINTER(Rmegb), C.c_uint16, C.POINTER(C.c_uint32)]
lib.rmegb_initialize.argtypes = [C.POINTER(Rmegb), C.c_uint32]

REG_POSITION, REG_FORCE, REG_STATUS = 0x1004, 0x1006, 0x1008

g = Rmegb()
if lib.rmegb_open(C.byref(g), b"COM5", 115200, 1) != 0:
    raise OSError(f"cannot open port, OS error {g.os_error}")
lib.rmegb_write(C.byref(g), REG_FORCE, 40)       # force 40 %
lib.rmegb_write(C.byref(g), REG_POSITION, 0)     # opening 0 %
status = C.c_uint32()
lib.rmegb_read(C.byref(g), REG_STATUS, C.byref(status))
lib.rmegb_close(C.byref(g))
```

If you change `struct rmegb` in `rmegb.h`, change `_fields_` to match.

## 4. Documents

Manufacturer documentation:

| Document | Contents |
|---|---|
| [RobustMotion RM-EGB gripper user manual](https://www.pololu.com/file/0J2112/RobustMotion%20RM-EGB%20gripper%20user%20manual.pdf) | Gripper specifications, wiring, I/O control and the Modbus RTU register map (section 4, pp. 13–19) used by this driver |
| [DSD TECH SH-U11F user guide](https://www.cncwiki.org/images/b/b6/DSD_Tech_SH-U11F_User_Guide.pdf) | USB to RS-485/RS-422 adapter: terminals, wiring and drivers |

These documents are the property of their respective manufacturers and are
not covered by the MIT License below.

## 5. Disclaimer

This software was generated and tested with Claude Code (Anthropic), based on
the information in the [RobustMotion RM-EGB user manual](https://www.pololu.com/file/0J2112/RobustMotion%20RM-EGB%20gripper%20user%20manual.pdf),
and has been reviewed by a human. It has not been independently validated on
every hardware and software configuration.

The software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose and non-infringement. In no event shall the
authors or copyright holders be liable for any claim, damages or other
liability, including damage to equipment or property, injury, loss of data
or production, or any other loss or inconvenience, whether in an action of
contract, tort or otherwise, arising from, out of or in connection with the
software or its use.

This software is not a safety-rated component. The user is solely
responsible for verifying its correct operation in the target system and for
providing appropriate safeguards (emergency stop, guarding, safe-torque-off
and the like) in accordance with the safety standards that apply to the
installation.

## 6. Licensing

This project is released under the MIT License:

```
MIT License

Copyright (c) 2026 Oleksandr Matiash

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 7. Copyright

Copyright © 2026 Oleksandr Matiash. All rights reserved except as granted by
the MIT License above.

RobustMotion and RM-EGB are trademarks of their respective owner. This
project is not affiliated with or endorsed by RobustMotion; the register map
is taken from the publicly available RM-EGB user manual.
