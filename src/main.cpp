#include <Arduino.h>
#include "platform.h"
#include "driver/mcp2510.h"
#include "log.h"

MCP2510 can_driver(PLATFORM_PIN_CAN_CS, 1, 16E6, 250E4);
void setup()
{
  // SerialUSB.begin(115200);
  //   put your setup code here, to run once:
  platform_init();

  // initialize can
  if (can_driver.begin(250E3))
  {
    log("CAN init failed");
  }
}

void loop()
{
  // SerialUSB.println("loop");
  //  put your main code here, to run repeatedly:
  if (can_driver.begin_packet(0x5a5, 0, false) < 0)
  {
    log("failed to start packet");
  }
  can_driver.write(0);
  can_driver.write(1);
  can_driver.write(2);
  can_driver.write(4);
  can_driver.write(8);
  can_driver.write(16);
  can_driver.write(32);
  can_driver.write(64);
  int error_code = can_driver.end_packet();
  if (error_code < 0)
  {
    log("failed to send message :(");
    log("error code: ");
    log(error_code);
  }

  delay(500);
}