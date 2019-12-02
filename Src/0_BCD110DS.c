/*****************************************************************************
* File Name          : 0_BCD110DS.c
* Description        : BCD110DS Bluetooth Module Driver (SENA, CHIPSEN)
******************************************************************************
* Design by Sung Jin, Park
* Create Date : 2019. 02. 18.
* Update
******************************************************************************/
#include "0_BCD110DS.h"

uint8_t sendData[35];

uint8_t doAT(uint8_t* rxDataBuffer)
{
	uint8_t temp[20] = {0};	
	uint8_t result;
	uint8_t count = 0;
	uint8_t check = 0;
	uint8_t i;

	while(TRUE)
	{
		result = FALSE;
		
		CopyToArray(sendData, (uint8_t*)AT_AT, 3, 35);
		HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
		HAL_UART_Transmit(&huart2, sendData, 3, 100);
		osDelay(300);
		
		//count = BtRxQueue.count;

		if(count == 0)
		{
			check++;
			if(check > 5)
			{
				return FALSE;
			}
		}
		else if(count >= 6)
		{
			for(i = 0; i < count; i++)
			{
				//temp[i] = RxQueue_Recive(&BtRxQueue);
			}
	
			for(i = 0; i < 5; i++)
			{
				if((temp[i] == 'O') && (temp[i + 1] == 'K'))
				{
						result = TRUE;
						break;
				}
			}
	
			if(result == TRUE)
			{
				osDelay(1);
				return TRUE;
			}
		}
	}
}


uint8_t doAT_PLUS(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_PLUS, 4, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 4, 100);
	osDelay(100);

	return TRUE;
}

uint8_t doAT_ATH(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_ATH, 4, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 4, 100);
	osDelay(100);

	return TRUE;
}

uint8_t doAT_ATZ(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_ATZ, 4, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 4, 100);
	osDelay(2000);
	//같은 명령 두번 안주면 보레이트 안바ㄱ궈짐. 
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 4, 100);
	osDelay(2000);

	return TRUE;
}


uint8_t doAT_ATO(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_ATO, 4, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 4, 100);
	osDelay(100);

	return TRUE;
}

uint8_t doAT_BTINFO(uint8_t* rxDataBuffer)
{
	uint8_t count = 0;
	uint8_t temp[50] = {0};
	uint8_t result = TRUE;	
	uint8_t i;

	CopyToArray(sendData, (uint8_t*)AT_BTINFO, 11, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 11, 100);
	osDelay(300);
	
	//count = BtRxQueue.count;

	if(count >= 30)
	{
		for(i = 0; i < count; i++)
		{
			//temp[i] = RxQueue_Recive(&BtRxQueue);
		}

		for(i = 0; i < count; i++)
		{
			if((temp[i] == 'C') && (temp[i + 1] == 'O') && (temp[i + 2] == 'N'))
			{
					result = CONNECT;
					break;
			}

			if((temp[i] == 'S') && (temp[i + 1] == 'T') && (temp[i + 2] == 'A'))
			{
					result = STANDBY;
					break;
			}
		}
	}
	
	return result;
}

uint8_t doAT_BTNAME(uint8_t* rxDataBuffer, uint8_t* newName)
{	
	uint8_t result;
	uint8_t temp[20] = {0};
	uint8_t count = 0;
	uint8_t i;

	CopyToArray(sendData, (uint8_t*)AT_BTNAME, 32, 35);
	CopyToArray(&sendData[18], newName, 12, 12);
	
	while(TRUE)
	{ 			
		result = FALSE;
		
		HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
		HAL_UART_Transmit(&huart2, sendData, 32, 100);
		osDelay(300);
	
		//count = BtRxQueue.count;
	
		if(count >= 6)
		{
			for(i = 0; i < count; i++)
			{
				//temp[i] = RxQueue_Recive(&BtRxQueue);
			}
	
			for(i = 0; i < 5; i++)
			{
				if((temp[i] == 'O') && (temp[i + 1] == 'K'))
				{
						result = TRUE;
						break;
				}
			}
	
			if(result == TRUE)
			{
				osDelay(1);
				break;
			}
		}
	}

	return TRUE;
}

uint8_t doAT_UARTCONFIR_921600(uint8_t* rxDataBuffer)
{
	uint8_t temp[20] = {0};	
	uint8_t result;
	uint8_t count = 0;
	uint8_t i;

	while(TRUE)
	{
		result = FALSE;
		//for(i = 0; i < 20; i++) temp[i] = 0;
		
		CopyToArray(sendData, (uint8_t*)AT_UARTCONFIG_921600, 27, 35);
		HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
		HAL_UART_Transmit(&huart2, sendData, 28, 100);
		osDelay(200);
		
		//count = BtRxQueue.count;
	
		if(count >= 6)
		{
			for(i = 0; i < count; i++)
			{
				//temp[i] = RxQueue_Recive(&BtRxQueue);
			}
	
			for(i = 0; i < 5; i++)
			{
				if((temp[i] == 'O') && (temp[i + 1] == 'K'))
				{
						result = TRUE;
						break;
				}
			}
	
			if(result == TRUE)
			{
				osDelay(1);
				break;
			}
		}
	}
	
	return TRUE;
}


uint8_t doAT_BTCANCEL(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_BTCANCEL, 12, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 12, 100);
	osDelay(500);

	return TRUE;
}


uint8_t doAT_BTMODE_3(uint8_t* rxDataBuffer)
{
	uint8_t temp[20] = {0};	
	uint8_t result;
	uint8_t count = 0;
	uint8_t error_count = 0;
	uint8_t i;

	while(TRUE)
	{
		result = FALSE;

		CopyToArray(sendData, (uint8_t*)AT_BTMODE3, 12, 35);
		HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
		HAL_UART_Transmit(&huart2, sendData, 12, 100);
		osDelay(300);
		
		//count = BtRxQueue.count;
			
		if(count >= 6)
		{
			for(i = 0; i < count; i++)
			{
				//temp[i] = RxQueue_Recive(&BtRxQueue);
			}
	
			for(i = 0; i < 5; i++)
			{
				if((temp[i] == 'O') && (temp[i + 1] == 'K'))
				{
						result = TRUE;
						break;
				}
			}
	
			if(result == TRUE)
			{
				osDelay(1);
				break;
			}
			else
			{
				error_count++;
				if(error_count > 20)
				{
					return FALSE;
				}
			}
		}
	}			

	return TRUE;
}


uint8_t doAT_FACTORY_RESET(uint8_t* rxDataBuffer)
{
	CopyToArray(sendData, (uint8_t*)AT_FACTORY_RESET, 5, 35);
	HAL_UART_Receive_IT(&huart2, rxDataBuffer, 2);
	HAL_UART_Transmit(&huart2, sendData, 12, 100);
	osDelay(300);

	return TRUE;
}

