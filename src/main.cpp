#include <Arduino.h>
#include <hardware/irq.h>
#include "platform/platform.h"
#include "driver/can/spi_mcp2510.h"
#include "platform/log.h"
#include "app.h"

enum class AppState
{
    SETUP,
    USER_SETUP,
    LOOP,
    FATAL,
};

static AppState g_state = AppState::SETUP;

void setup()
{
    platform_init();
}

void global_setup()
{
    // Global setup
    g_state = AppState::USER_SETUP;
}

void user_setup()
{
    // User setup
    int err = app_setup();
    if (err)
    {
        platform_set_status(err);
        g_state = AppState::FATAL;
    }
    else
    {
        g_state = AppState::LOOP;
    }
}

void user_loop()
{
    int err = app_loop();
    if (err)
    {
        platform_set_status(err);
        g_state = AppState::FATAL;
    }
}

void global_error()
{
    // TODO: Log error if possible
    while (2)
    {
        // wait for watchdog restart
    }
}

void loop()
{
    platform_preloop();
    switch (g_state)
    {
    case AppState::SETUP:
        global_setup();
        break;
    case AppState::USER_SETUP:
        user_setup();
        break;
    case AppState::LOOP:
        user_loop();
        break;
    case AppState::FATAL:
        global_error();
        break;
    }
    platform_postloop();
}