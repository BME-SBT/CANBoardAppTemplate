//
// Created by Barrow099 on 2023. 04. 06..
//

#include "lib/ringbuffer.h"
#include <stdio.h>
#include <stdlib.h>

void setUp() {}
void tearDown() {}

#define TEST_ASSERT(cond)                                                      \
    if (!(cond)) {                                                             \
        printf("assertion failed: ##cond");                                    \
        exit(1);                                                               \
    }
#define TEST_ASSERT_TRUE(val) TEST_ASSERT(val == true)
#define TEST_ASSERT_FALSE(val) TEST_ASSERT(val == false)

void test_ringbuf_init() {
    UnsafeRingBuffer<int, 12> buffer;
    printf("full: %d\n", buffer.is_full());
    printf("empty: %d\n", buffer.is_empty());
}

int main(int argc, char **argv) { test_ringbuf_init(); }
