#include "app.h"

Measurement motor_rpm((u16)0b00000010101,10,new RPM_Datatype);

PlatformStatus app_setup() {
    PLATFORM_MEASUREMENTS.add_measurement(&motor_rpm);

    return PlatformStatus::STATUS_OK;
}

u16 counter = 0;

PlatformStatus app_loop() {
    motor_rpm.set_value(counter++);
    delayMicroseconds(10);
    return PlatformStatus::STATUS_OK;
}