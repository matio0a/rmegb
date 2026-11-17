@echo off
rem build_dll.bat - build the gripper driver as lib\rmegb.dll for Windows,
rem                 for use from Python (ctypes) and other languages.
rem
rem   Uses MSVC (cl) if it is available, that is when run from an
rem   "x64 Native Tools Command Prompt for VS"; otherwise MinGW-w64 (gcc).
rem
rem   Bitness: build 64-bit for a 64-bit Python (almost always the case).
rem   Use the x64 prompt for MSVC, or a 64-bit MinGW-w64 gcc.

setlocal
rem run from the folder of this script
cd /d "%~dp0"
rem output folder
if not exist lib mkdir lib

rem is MSVC's compiler on the PATH?
where cl >nul 2>nul
if %errorlevel%==0 goto msvc
rem is MinGW's gcc on the PATH?
where gcc >nul 2>nul
if %errorlevel%==0 goto mingw
echo error: neither cl (MSVC) nor gcc (MinGW-w64) found on PATH
exit /b 1

:msvc
rem /nologo        no banner
rem /O2            optimize
rem /LD            build a DLL (also writes lib\rmegb.lib import library)
rem /DRMEGB_DLL    marks the six API functions __declspec(dllexport), see
rem                rmegb.h; without it the DLL would export nothing
rem /I include     find rmegb.h
rem /Fo:lib\       put object files in lib\
rem /Fe:...        name of the DLL
rem kernel32.lib (serial port and timer functions) is linked by default
cl /nologo /O2 /LD /DRMEGB_DLL /I include src\rmegb.c src\rmegb_serial_win32.c /Fo:lib\ /Fe:lib\rmegb.dll
if errorlevel 1 exit /b 1
echo built lib\rmegb.dll with MSVC
echo exported functions:
rem list the exports; should show the six rmegb_* functions
dumpbin /nologo /exports lib\rmegb.dll | findstr rmegb_
goto end

:mingw
rem -O2              optimize
rem -shared          build a DLL
rem -DRMEGB_DLL      marks the API functions dllexport, see rmegb.h; MinGW
rem                  then exports only those (instead of every function)
rem -Iinclude        find rmegb.h
rem -static-libgcc   no dependency on libgcc_s DLL, so rmegb.dll runs on
rem                  machines without MinGW installed
gcc -O2 -shared -DRMEGB_DLL -Iinclude src/rmegb.c src/rmegb_serial_win32.c -static-libgcc -o lib/rmegb.dll
if errorlevel 1 exit /b 1
echo built lib\rmegb.dll with MinGW
echo exported functions:
rem list the exports; should show the six rmegb_* functions
objdump -p lib\rmegb.dll | findstr rmegb_

:end
endlocal
