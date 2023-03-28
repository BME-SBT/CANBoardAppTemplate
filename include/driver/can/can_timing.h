#ifndef CAN_TIMING_H
#define CAN_TIMING_H

#include "platform/platform.h"

static const struct
{
    long clockFrequency;
    long baudRate;
    u8 cnf[3];
} mcp251x_cnf_mapper[] = {
    {(long)8E6, (long)1000E3, {0x00, 0x80, 0x00}},
    {(long)8E6, (long)500E3, {0x00, 0x90, 0x02}},
    {(long)8E6, (long)250E3, {0x00, 0xb1, 0x05}},
    {(long)8E6, (long)200E3, {0x00, 0xb4, 0x06}},
    {(long)8E6, (long)125E3, {0x01, 0xb1, 0x05}},
    {(long)8E6, (long)100E3, {0x01, 0xb4, 0x06}},
    {(long)8E6, (long)80E3, {0x01, 0xbf, 0x07}},
    {(long)8E6, (long)50E3, {0x03, 0xb4, 0x06}},
    {(long)8E6, (long)40E3, {0x03, 0xbf, 0x07}},
    {(long)8E6, (long)20E3, {0x07, 0xbf, 0x07}},
    {(long)8E6, (long)10E3, {0x0f, 0xbf, 0x07}},
    {(long)8E6, (long)5E3, {0x1f, 0xbf, 0x07}},

    {(long)16E6, (long)1000E3, {0x00, 0xd0, 0x82}},
    {(long)16E6, (long)500E3, {0x00, 0xf0, 0x86}},
    {(long)16E6, (long)250E3, {0x41, 0xf1, 0x85}},
    {(long)16E6, (long)200E3, {0x01, 0xfa, 0x87}},
    {(long)16E6, (long)125E3, {0x03, 0xf0, 0x86}},
    {(long)16E6, (long)100E3, {0x03, 0xfa, 0x87}},
    {(long)16E6, (long)80E3, {0x03, 0xff, 0x87}},
    {(long)16E6, (long)50E3, {0x07, 0xfa, 0x87}},
    {(long)16E6, (long)40E3, {0x07, 0xff, 0x87}},
    {(long)16E6, (long)20E3, {0x0f, 0xff, 0x87}},
    {(long)16E6, (long)10E3, {0x1f, 0xff, 0x87}},
    {(long)16E6, (long)5E3, {0x3f, 0xff, 0x87}},
};

#endif