USE_RTT_FOR_DEBUG = 1

RTT_C_SOURCES = \
Lib/RTT/SEGGER_RTT.c \
Lib/RTT/SEGGER_RTT_printf.c \
Lib/Syscalls/SEGGER_RTT_Syscalls_GCC.c

RTT_C_INCLUDES = \
-ILib/RTT

APP_C_SOURCES = \
Src/0_16Segment.c \
Src/0_BCD110DS.c \
Src/0_SdCard.c \
Src/0_SensorCal.c \
Src/0_soonFlashMemory.c \
Src/0_StartDisplayTask.c \
Src/0_StartRateTask.c \
Src/0_StartSlotUartTask.c \
Src/external_uart_task.c \
Src/internal_uart_task.c \
Src/0_UartCallback.c \
Src/0_Util.c

