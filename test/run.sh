#!/usr/bin/env sh

gcc -o ringbuffer_test ringbuffer_test.cpp -I../include -I../lib/core/include || exit
./ringbuffer_test
rm -rf ./ringbuffer_test