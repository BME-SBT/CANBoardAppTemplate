#include "platform.h"
#include "log.h"

MbedSPI PLATFORM_SPI(PLATFORM_PIN_SPI_MISO, PLATFORM_PIN_SPI_MOSI, PLATFORM_PIN_SPI_SCK);

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

    PLATFORM_SPI.begin();
}