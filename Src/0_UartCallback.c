#include "0_UartCallback.h"

/*********************************************************************
*	Callback Function
*	2byte 입력시 한번 인터럽트 발생 
*	dma로 입력 받을 경우 16 바이트 단위 입력으로 해야해서 bt module 의 
*	응답을 받아서 처리 하기 힘듬 
**********************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    uint8_t i;
    
	if(huart->Instance == USART2)       //Slot Interface & Bluetooth
	{
        osSemaphoreRelease(CountingSemSlaveRxHandle);
	}
    else if(huart->Instance == USART1)
    {
        osSemaphoreRelease(BinarySem485RxHandle);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)   //Slot Interface & Bluetooth
    {        
        osSemaphoreRelease(BinarySemSlaveTxHandle);
    }
    else if(huart->Instance == USART1)   //
    {
        osSemaphoreRelease(CountingSem485TxHandle);
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
    }
}

