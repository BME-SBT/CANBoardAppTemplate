#include "platform/platform.h"
#include "platform/log.h"

u8 status_last_error = 0;

void platform_init()
{
    // initialize platform
    SerialUSB.begin(115200);
    delay(500);
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    log("Initializing platform...");
}