#include "0_UartCallback.h"


/*********************************************************************
*	Callback Function
*	2byte 입력시 한번 인터럽트 발생 
*	dma로 입력 받을 경우 16 바이트 단위 입력으로 해야해서 bt module 의 
*	응답을 받아서 처리 하기 힘듬 
**********************************************************************/

extern int ext_tx_completed;
extern int int_tx_completed;
extern int int_rx_completed;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if (huart->Instance == USART2)       //Slot Interface & Bluetooth
	{
		/* int_rx_completed = 1; */
	}
	else if(huart->Instance == USART1)
	{
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)   //Slot Interface & Bluetooth
	{
		int_tx_completed = 1;
		/* HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET); */
		/* HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_RESET); */
	}
	else if (huart->Instance == USART1)
	{
		ext_tx_completed = 1;
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
	}
}
