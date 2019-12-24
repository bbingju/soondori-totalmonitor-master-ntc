#pragma once

#ifdef USE_RTT_FOR_DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef USE_RTT_FOR_DEBUG
#define DBG_LOG(f_, ...)          SEGGER_RTT_printf(0, (f_), ##__VA_ARGS__)
#define DBG_DUMP(d, len)                                                       \
    do {                                                                       \
        int row = len / 16;                                                    \
        SEGGER_RTT_PutChar(0, '\n');                                           \
        for (int j = 0; j <= row; j++) {                                       \
            SEGGER_RTT_PutChar(0, '\t');                                       \
            for (int i = 0; i < 16; i++) {                                     \
                if (len == j * 16 + i)                                         \
                    break;                                                     \
                if (i == 0)                                                    \
                    SEGGER_RTT_printf(0, "%02d: %02x ", j + 1,                 \
                                      *(((uint8_t *)(d)) + (j * 16 + i)));     \
                else                                                           \
                    SEGGER_RTT_printf(0, "%02x ",                              \
                                      *(((uint8_t *)(d)) + (j * 16 + i)));     \
            }                                                                  \
            SEGGER_RTT_PutChar(0, '\n');                                       \
        }                                                                      \
    } while (0)

#else
#define DBG_LOG(f_, ...)
#define DBG_DUMP(stream, len)
#endif

