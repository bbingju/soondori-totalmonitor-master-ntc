#ifndef __SOON_16SEGMENT_H__
#define __SOON_16SEGMENT_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
//#include "main.h"
//#include "0_util.h"


#ifndef __STM32F4xx_HAL_H
  #include "stm32f4xx_hal.h"
#endif

#ifndef _STDINT
	#include "stdint.h"
#endif


typedef enum {
	SSP_NULL = 0,
	SSP_UNDERBAR,
	SSP_HYPHEN,
	SSP_BACKSLASH,
	SSP_OR,
	SSP_SLASH,
	SSP_PLUS,
	SSP_STAR
}SEGMENT_SPECIAL_FONT;




void SegmentDisplay(uint8_t digit, uint16_t data);
uint16_t doTextToDigitdata(char text);
uint16_t doNumberToDigitdata(SEGMENT_SPECIAL_FONT number);


#endif

