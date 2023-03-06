#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED 1

#ifdef LOG_ENABLED
#define log(msg)                            \
    do                                      \
    {                                       \
        SerialUSB.print("[");               \
        SerialUSB.print(millis() / 1000.0); \
        SerialUSB.print("] ");              \
        SerialUSB.print(__FILE__);          \
        SerialUSB.print(":");               \
        SerialUSB.print(__LINE__);          \
        SerialUSB.print(": ");              \
        SerialUSB.println(msg);             \
    } while (0)
#else
#define log(msg)
#endif

#endif