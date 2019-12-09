#include "cmsis_os.h"
#include "0_UartCallback.h"
#include "internal_rx_task.h"

/*********************************************************************
*	Callback Function
*	2byte 입력시 한번 인터럽트 발생 
*	dma로 입력 받을 경우 16 바이트 단위 입력으로 해야해서 bt module 의 
*	응답을 받아서 처리 하기 힘듬 
**********************************************************************/

/* extern struct internal_tx_msg_s *tx_received; */
/* extern osMailQId (internal_tx_pool_q_id); */

extern int ext_tx_completed;
extern int int_tx_completed;
extern int int_rx_completed;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USART2)       //Slot Interface & Bluetooth
    {
        /* internal_rx_msg_push(huart->pRxBuffPtr, huart->RxXferSize); */
        /* if (tx_received) { */
        /*     osMailFree(internal_tx_pool_q_id, tx_received); */
        /*     tx_received = NULL; */
        /* } */
        /* osSemaphoreRelease(CountingSemSlaveRxHandle); */

        /* push_internal_resp(huart->pRxBuffPtr, huart->RxXferSize); */
        int_rx_completed = 1;
    }
    else if(huart->Instance == USART1)
    {
        push_external_rx(huart->pRxBuffPtr, huart->RxXferSize);
        /* osSemaphoreRelease(BinarySem485RxHandle); */
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)   //Slot Interface & Bluetooth
    {
        int_tx_completed = 1;
        /* osSemaphoreRelease(BinarySemSlaveTxHandle); */
    }
    else if(huart->Instance == USART1)   //
    {
        ext_tx_completed = 1;
        /* osSemaphoreRelease(CountingSem485TxHandle); */
	/* HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); */
    }
}

