cmake_minimum_required(VERSION 3.0)
file(GLOB SOURCES *.c)
set(CMAKE_C_FLAGS "-O3 -Wall")
add_library(emu8950 STATIC ${SOURCES})
