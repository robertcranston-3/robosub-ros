#!/bin/bash
x86_64-w64-mingw32-g++ -shared -o simExtLibTest.dll -static -static-libgcc -static-libstdc++ -I include -I include/stack common/*.cpp common/stack/*.cpp simExtLibTest.c -fPIC -g -lshlwapi -lwinmm -lws2_32