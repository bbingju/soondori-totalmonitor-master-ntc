#include "0_UartCallback.h"

/*********************************************************************
*	Callback Function
*	2byte �Է½� �ѹ� ���ͷ�Ʈ �߻� 
*	dma�� �Է� ���� ��� 16 ����Ʈ ���� �Է����� �ؾ��ؼ� bt module �� 
*	������ �޾Ƽ� ó�� �ϱ� ���� 
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

