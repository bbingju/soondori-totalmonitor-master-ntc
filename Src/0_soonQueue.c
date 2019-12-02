#include "0_soonQueue.h"
#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

#ifndef __STM32F4xx_HAL_SD_H
//  #include "stm32f4xx_hal_sd.h"
#endif 


/*********************************************************************
**********************************************************************
*	Bluetooth 통신용 Queue 함수들 
**********************************************************************
**********************************************************************/

/*********************************************************************
*	TxQueue_empty
*	큐가 비어 잇ㅅ는지 확인 하는 함수
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
uint8_t TxQueue_empty(TX_QUEUE_STRUCT *q)
{
	if( q->head == q->tail)
		return TRUE;
	return FALSE;
}

/*********************************************************************
*	TxQueue_Init
*	큐가 초기화 함수 
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
void TxQueue_Init(TX_QUEUE_STRUCT *q)
{
	q->size = UART_TX_BUF_MAX;
	q->count = 0;
	q->head = 0;
	q->tail = 0;
}

/*********************************************************************
*	TxQueue_Send
*	큐로 데이터를 입력, 1byte 단위로 입력 
*	*q		: 사용 되는 Queue 주소 
*	data	: 입려되는 데이터 
**********************************************************************/
void TxQueue_Send(TX_QUEUE_STRUCT *q, uint8_t data)
{
	q->ar[q->head++]	= data;
	q->head = q->head % UART_TX_BUF_MAX;
	q->count++;
}

/*********************************************************************
*	TxQueue_Recive
*	큐로 데이터를 출력, 1byte 단위로 출력
*	*q		: 사용 되는 Queue 주소 
*	return	: 1byte 단위로 출력 
**********************************************************************/
uint8_t TxQueue_Recive(TX_QUEUE_STRUCT *q)
{
	uint8_t data;
	if( TxQueue_empty(q) == TRUE)
		return -1;

	data	= q->ar[q->tail++];
	q->tail = q->tail % UART_TX_BUF_MAX;
	q->count--;
	return data;
}

/*********************************************************************
*	RxQueue_empty
*	큐가 비어 잇ㅅ는지 확인 하는 함수
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
uint8_t RxQueue_empty(RX_QUEUE_STRUCT *q)
{
	if( q->head == q->tail)
		return TRUE;
	return FALSE;
}

/*********************************************************************
*	RxQueue_Init
*	큐가 초기화 함수 
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
void RxQueue_Init(RX_QUEUE_STRUCT *q)
{
	q->size = UART_RX_BUF_MAX;
	q->count = 0;
	q->head = 0;
	q->tail = 0;
}

/*********************************************************************
*	RxQueue_Send
*	큐로 데이터를 입력, 1byte 단위로 입력 
*	*q		: 사용 되는 Queue 주소 
*	data	: 입려되는 데이터 
**********************************************************************/
void RxQueue_Send(RX_QUEUE_STRUCT *q, uint8_t data)
{
	q->ar[q->head++]	= data;
	q->head = q->head % UART_RX_BUF_MAX;
	q->count++;
}

/*********************************************************************
*	RxQueue_Recive
*	큐로 데이터를 출력, 1byte 단위로 출력
*	*q		: 사용 되는 Queue 주소 
*	return	: 1byte 단위로 출력 
**********************************************************************/
uint8_t RxQueue_Recive(RX_QUEUE_STRUCT *q)
{
	uint8_t data;
	if( RxQueue_empty(q) == TRUE)
		return -1;

	data	= q->ar[q->tail++];
	q->tail = q->tail % UART_RX_BUF_MAX;
	q->count--;
	return data;
}

/*********************************************************************
*	RxQueue_Clear
*	큐데이터 클리어 
*	*q		: 사용 되는 Queue 주소 
**********************************************************************/
void RxQueue_Clear(RX_QUEUE_STRUCT *q)
{
	while(RxQueue_empty(q) == FALSE)
	{
		RxQueue_Recive(q);
	}
}

uint8_t RxQueue_Count(RX_QUEUE_STRUCT *q)
{
	return q->count;
}


/*********************************************************************
*	Rs485RxQueue_empty
*	큐가 비어 잇ㅅ는지 확인 하는 함수
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
uint8_t Rs485TxQueue_empty(RS_485_TX_QUEUE_STRUCT *q)
{
	if( q->head == q->tail)
		return TRUE;
	return FALSE;
}

/*********************************************************************
*	Rs485RxQueue_Init
*	큐가 초기화 함수 
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
void Rs485TxQueue_Init(RS_485_TX_QUEUE_STRUCT *q)
{
	q->size = RS_485_TX_BUF_MAX;
	q->count = 0;
	q->head = 0;
	q->tail = 0;
}

/*********************************************************************
*	Rs485RxQueue_Send
*	큐로 데이터를 입력, 1byte 단위로 입력 
*	*q		: 사용 되는 Queue 주소 
*	data	: 입려되는 데이터 
**********************************************************************/
void Rs485TxQueue_Send(RS_485_TX_QUEUE_STRUCT *q, uint8_t data)
{
	q->ar[q->head++]	= data;
	q->head = q->head % RS_485_TX_BUF_MAX;
	q->count++;
}

/*********************************************************************
*	Rs485RxQueue_Recive
*	큐로 데이터를 출력, 1byte 단위로 출력
*	*q		: 사용 되는 Queue 주소 
*	return	: 1byte 단위로 출력 
**********************************************************************/
uint8_t Rs485TxQueue_Recive(RS_485_TX_QUEUE_STRUCT *q)
{
	uint8_t data;
	if( Rs485TxQueue_empty(q) == TRUE)
		return -1;

	data	= q->ar[q->tail++];
	q->tail = q->tail % RS_485_TX_BUF_MAX;
	q->count--;
	return data;
}

/*********************************************************************
*	Rs485RxQueue_empty
*	큐가 비어 잇ㅅ는지 확인 하는 함수
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
uint8_t Rs485RxQueue_empty(RS_485_RX_QUEUE_STRUCT *q)
{
	if( q->head == q->tail)
		return TRUE;
	return FALSE;
}

/*********************************************************************
*	Rs485RxQueue_Init
*	큐가 초기화 함수 
*	*q	: 사용 되는 Queue 주소 
**********************************************************************/
void Rs485RTxQueue_Init(RS_485_RX_QUEUE_STRUCT *q)
{
	q->size = RS_485_RX_BUF_MAX;
	q->count = 0;
	q->head = 0;
	q->tail = 0;
}

/*********************************************************************
*	Rs485RxQueue_Send
*	큐로 데이터를 입력, 1byte 단위로 입력 
*	*q		: 사용 되는 Queue 주소 
*	data	: 입려되는 데이터 
**********************************************************************/
void Rs485RxQueue_Send(RS_485_RX_QUEUE_STRUCT *q, uint8_t data)
{
	q->ar[q->head++]	= data;
	q->head = q->head % RS_485_RX_BUF_MAX;
	q->count++;
}

/*********************************************************************
*	Rs485RxQueue_Recive
*	큐로 데이터를 출력, 1byte 단위로 출력
*	*q		: 사용 되는 Queue 주소 
*	return	: 1byte 단위로 출력 
**********************************************************************/
uint8_t Rs485RxQueue_Recive(RS_485_RX_QUEUE_STRUCT *q)
{
	uint8_t data;
	if( Rs485RxQueue_empty(q) == TRUE)
		return -1;

	data	= q->ar[q->tail++];
	q->tail = q->tail % RS_485_RX_BUF_MAX;
	q->count--;
	return data;
}

/*********************************************************************
*	RxQueue_Clear
*	큐데이터 클리어 
*	*q		: 사용 되는 Queue 주소 
**********************************************************************/
void Rs485RxQueue_Clear(RS_485_RX_QUEUE_STRUCT *q)
{
	while(Rs485RxQueue_empty(q) == FALSE)
	{
		Rs485RxQueue_Recive(q);
	}
}

uint8_t Rs485RxQueue_Count(RS_485_RX_QUEUE_STRUCT *q)
{
	return q->count;
}


