#ifndef PLATFORM_H
#define PLATFORM_H

#include <Arduino.h>
#include <hardware/spi.h>
#include "lib/utils.h"
#include "lib/inttypes.h"
#include "platform/status.h"

#define PLATFORM_PIN_SPI_SCK 2
#define PLATFORM_CAN_SPI_BAUD 1E6 // 1 MHz
#define PLATFORM_PIN_SPI_MOSI 3
#define PLATFORM_PIN_SPI_MISO 4
#define PLATFORM_PIN_CAN_CS 5
#define PLATFORM_PIN_CAN_INT 1

#define PLATFORM_CAN_SPI spi0

void platform_init();
void platform_set_status(u8 status);

void platform_preloop();
void platform_postloop();

extern u8 status_last_error;

#endif