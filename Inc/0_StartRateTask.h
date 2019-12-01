#ifndef __START_RATE_TASK_H__
#define __START_RATE_TASK_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "0_Util.h"

#ifndef __STM32F4xx_HAL_CONF_H
  #include "stm32f4xx_hal_conf.h"
#endif

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

#define BOARD_SENSOR_POSITION   (512)

//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartRateTask(void const * argument);
void DoSdCardFunction(void);
void DoMCUboardInfo(void);
void DoSlotInfo(uint8_t slot);
void DoChannelInfo(uint8_t slot);
void DoChannelValue(uint8_t slot);
void DoSdCardFreeSpace(void);
void DoSmpsCheck(void);


#endif

