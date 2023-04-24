#include "app.h"
#include <api/Common.h>
#include <cstdio>
#include <cstdlib>

#include "lib/inttypes.h"

PlatformStatus app_setup() {
    srand(micros());
    return PlatformStatus::STATUS_OK;
}
u64 counter = 0;


PlatformStatus app_loop() {
    LOG("app loop");
    u16 id = rand() % 0x7ff;
    CAN_Frame frame(id, reinterpret_cast<u8 *>(&counter), 8);
    LOG("sending frame...");
    PLATFORM_CAN.send(frame);
    counter += 1;
    id += 1;
    if (id > 0x7ff) {
        id = 0;
    }
    while (PLATFORM_CAN.available()) {
        __attribute__((unused)) CAN_Frame rv = PLATFORM_CAN.get_frame();
        //printf("got frame: %d %d\r\n", rv.standard_id, *reinterpret_cast<u64*>(rv.data));
    }

    return PlatformStatus::STATUS_OK;
}