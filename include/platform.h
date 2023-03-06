#ifndef PLATFORM_H
#define PLATFORM_H

#include <Arduino.h>
#include <SPI.h>
#include "utils.h"

#define PLATFORM_PIN_SPI_SCK 2
#define PLATFORM_PIN_SPI_MOSI 3
#define PLATFORM_PIN_SPI_MISO 4

#define PLATFORM_PIN_CAN_CS 5

extern MbedSPI PLATFORM_SPI;

void platform_init();

class SPIWriter
{
    MAKE_NOCOPY(SPIWriter);
    MAKE_NOMOVE(SPIWriter);

public:
    explicit SPIWriter(SPISettings settings, int cs_pin) : m_settings(settings), m_cs_pin(cs_pin)
    {
        PLATFORM_SPI.beginTransaction(settings);
        digitalWrite(m_cs_pin, LOW);
    }

    uint8_t transfer(uint8_t data)
    {
        return PLATFORM_SPI.transfer(data);
    }

    ~SPIWriter()
    {
        digitalWrite(m_cs_pin, HIGH);
        PLATFORM_SPI.endTransaction();
    }

private:
    SPISettings m_settings;
    int m_cs_pin;
};

#endif