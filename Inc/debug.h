#pragma once

#ifdef USE_RTT_FOR_DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef USE_RTT_FOR_DEBUG
#define DBG_LOG(f_, ...)          SEGGER_RTT_printf(0, (f_), ##__VA_ARGS__)
#define DBG_DUMP(d, len)  do {                                          \
        for (int i = 0; i < len; i++)                                   \
            SEGGER_RTT_printf(0, "%02x ", *(((uint8_t *) (d)) + i));    \
        SEGGER_RTT_PutChar(0, '\n');                                    \
    } while (0)

#else
#define DBG_LOG(f_, ...)
#define DBG_DUMP(stream, len)
#endif

