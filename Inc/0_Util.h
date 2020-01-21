#ifndef __UTIL_H__
#define __UTIL_H__


#ifndef _STDINT
  #include <stdint.h>
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "0_GlobalValue.h"
#include "0_GlobalDefine.h"
#include "main.h"

#define BUZZER_ON	HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_SET)
#define BUZZER_OFF 	HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_RESET)

#define DATA_FULL_LENGTH 	6


float ByteArrayToFloat(uint8_t *byteArray);
uint8_t CopyToArray(uint8_t* TagetArray, uint8_t* OriginalString, uint16_t CopyLength, uint16_t TotalLength);
void doMakeSendData(uint8_t* SendData, uint8_t Command, uint8_t* Data, uint8_t  DataLength, uint8_t BufferLength);
void doMakeSendSlotData(uint8_t* SendData, uint8_t SlotNumber, uint8_t Command, uint8_t* Data, uint8_t  DataLength,	uint8_t BufferLength);
void doMakeRealTimeData(uint8_t* SendData, uint8_t Command, uint8_t Option, uint8_t* Data);
void doMakeSend485Data(	uint8_t* SendData, uint8_t Command, uint8_t Option, uint8_t* Data, uint16_t datalen, uint16_t arraylen);
void doPlayBuzzer100ms(uint16_t count);
void doBuzzerEnable(uint8_t enable);
void doBuzzerPlay(uint16_t delay);
void doNOP(uint16_t count);
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len);
float midADC_float(float * inData);
/* void swap(float *a, float *b); */
void doRejectSlot(void);

#endif

