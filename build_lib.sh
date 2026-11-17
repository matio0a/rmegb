#!/bin/sh
# build_lib.sh - build the gripper driver as a shared library for Linux or
#                macOS, for use from Python (ctypes) and other languages.
#
#   ./build_lib.sh          -> lib/librmegb.so   (Linux)
#                              lib/librmegb.dylib (macOS)
#
# The library has the same bitness as the compiler (64-bit on any current
# system); the program loading it (e.g. Python) must be 64-bit too.

set -e                                      # stop at the first failing command
cd "$(dirname "$0")"                        # run from the folder of this script

if [ "$(uname)" = "Darwin" ]; then          # macOS?
    OUT=lib/librmegb.dylib                  # macOS shared library name
else                                        # Linux and other Unix
    OUT=lib/librmegb.so                     # Linux shared library name
fi

mkdir -p lib                                # output folder

# -O2                   optimize
# -fPIC                 position-independent code, required for a shared library
# -shared               produce a shared library instead of a program
# -fvisibility=hidden   export only functions marked RMEGB_API (see rmegb.h);
#                       the internal port_*() functions stay private
# -Iinclude             find rmegb.h
# no extra libraries needed: termios, poll and clock_gettime are in libc
${CC:-gcc} -O2 -fPIC -shared -fvisibility=hidden -Iinclude \
    src/rmegb.c src/rmegb_serial_posix.c \
    -o "$OUT"

echo "built $OUT"                           # report the result
echo "exported functions:"                  # show what other languages can call
if [ "$(uname)" = "Darwin" ]; then          # macOS symbol listing
    nm -gU "$OUT" | grep rmegb_             # global, defined symbols
else                                        # Linux symbol listing
    nm -D --defined-only "$OUT" | grep rmegb_  # dynamic, defined symbols
fi
