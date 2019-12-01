#ifndef __SOON_QUEUE_H__
#define __SOON_QUEUE_H__

#ifndef __STM32F4xx_HAL_H
  #include "stm32f4xx_hal.h"
#endif

#ifndef _STDINT
	#include "stdint.h"
#endif

#include "0_GlobalValue.h"

uint8_t TxQueue_empty(TX_QUEUE_STRUCT *q);
void TxQueue_Init(TX_QUEUE_STRUCT *q);
void TxQueue_Send(TX_QUEUE_STRUCT *q, uint8_t data);
uint8_t TxQueue_Recive(TX_QUEUE_STRUCT *q);
uint8_t RxQueue_empty(RX_QUEUE_STRUCT *q);
void RxQueue_Init(RX_QUEUE_STRUCT *q);
void RxQueue_Send(RX_QUEUE_STRUCT *q, uint8_t data);
uint8_t RxQueue_Recive(RX_QUEUE_STRUCT *q);
void RxQueue_Clear(RX_QUEUE_STRUCT *q);
uint8_t RxQueue_Count(RX_QUEUE_STRUCT *q);
uint8_t Rs485TxQueue_empty(RS_485_TX_QUEUE_STRUCT *q);
void Rs485TxQueue_Init(RS_485_TX_QUEUE_STRUCT *q);
void Rs485TxQueue_Send(RS_485_TX_QUEUE_STRUCT *q, uint8_t data);
uint8_t Rs485TxQueue_Recive(RS_485_TX_QUEUE_STRUCT *q);
uint8_t Rs485RxQueue_empty(RS_485_RX_QUEUE_STRUCT *q);
void Rs485RTxQueue_Init(RS_485_RX_QUEUE_STRUCT *q);
void Rs485RxQueue_Send(RS_485_RX_QUEUE_STRUCT *q, uint8_t data);
uint8_t Rs485RxQueue_Recive(RS_485_RX_QUEUE_STRUCT *q);
void Rs485RxQueue_Clear(RS_485_RX_QUEUE_STRUCT *q);
uint8_t Rs485RxQueue_Count(RS_485_RX_QUEUE_STRUCT *q);

#endif
