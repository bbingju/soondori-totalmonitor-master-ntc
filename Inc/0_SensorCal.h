#ifndef __SENSOR_CAL_H__
#define __SENSOR_CAL_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"


#ifndef __STM32F4xx_HAL_H
  #include "stm32f4xx_hal.h"
#endif

#ifndef _STDINT
	#include "stdint.h"
#endif




float Calc_BD_Temp(uint32_t val);
float Calc_BD_Humi(uint32_t val);
float Calc_Temp_NTC(uint32_t val);
float Calc_Temp_RTD(uint32_t val);


#endif
