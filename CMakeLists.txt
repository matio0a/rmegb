cmake_minimum_required(VERSION 3.13)
project(rmegb C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(WIN32)
  set(PORT_SRC src/rmegb_serial_win32.c)
else()
  set(PORT_SRC src/rmegb_serial_posix.c)
endif()

add_library(rmegb STATIC src/rmegb.c ${PORT_SRC})
target_include_directories(rmegb PUBLIC include)
if(MSVC)
  target_compile_definitions(rmegb PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

add_executable(grip examples/grip.c)
target_link_libraries(grip PRIVATE rmegb)

add_executable(sequence examples/sequence.c)
target_link_libraries(sequence PRIVATE rmegb)

# Offline test: rmegb.c against a fake port, no hardware needed.
enable_testing()
add_executable(test_rmegb tests/test_rmegb.c src/rmegb.c)
target_include_directories(test_rmegb PRIVATE include)
add_test(NAME rmegb COMMAND test_rmegb)
