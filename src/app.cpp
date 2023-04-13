#include "app.h"

#include "lib/inttypes.h"

PlatformStatus app_setup() { return PlatformStatus::STATUS_OK; }
u64 counter = 0;
u16 id = 0;
u64 last_sent = 0;
u64 send_delay = 100;

PlatformStatus app_loop() {

    CAN_Frame frame(id, reinterpret_cast<u8 *>(&counter), 8);

    PLATFORM_CAN.send(frame);
    PLATFORM_CAN.send(frame);
    PLATFORM_CAN.send(frame);
    PLATFORM_CAN.send(frame);

    counter += 1;
    id += 1;
    if (id > 0x7ff) {
        id = 0;
    }
    while (PLATFORM_CAN.available()) {
        __attribute__((unused)) CAN_Frame rv = PLATFORM_CAN.get_frame();
        LOGF("got frame: %d %p", rv.standard_id, *(uint64_t*) (rv.data));
    }

    return PlatformStatus::STATUS_OK;
}