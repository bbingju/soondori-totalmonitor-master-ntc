#ifndef __START_DISPLAY_TASK_H__
#define __START_DISPLAY_TASK_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "0_Util.h"

#ifndef __STM32F4xx_HAL_CONF_H
  #include "stm32f4xx_hal_conf.h"
#endif

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"


//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartDisplayTask(void const * argument);
void DoDisplayModeChange(void);
void doSegmentDisplay(uint8_t quarterSec);
void doBatteryVoltageCheck(void);
void doMainBoardSensorCheck(void);
void doModeButton(void);

#endif

