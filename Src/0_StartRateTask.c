#include "0_StartRateTask.h"
#include "0_GlobalValue.h"
#include "0_StartRs485Task.h"
#include "FF_gen_drv.h"
#include "Sd_diskio.h"
#include "fatfs.h"
#include "0_SdCard.h"

#include <stdio.h>              /* sprintf() */

/*********************************************************************
*	Private variables
**********************************************************************/
uint8_t 		RealTimeSendData[160] = { 0 };
uint32_t 		nameCount = 0;
uint8_t 		tempData[1];

SYSTEM_TIME  	SysTime;
SD_CARD_VALUE	sdValue;

extern Disk_drvTypeDef disk;

/*********************************************************************
*	StartDisplayTask
*	16-SEGMENT, LED, BUTTON INPUT ���� ó�� �ϴ� TASK
**********************************************************************/
void StartRateTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	portTickType	xLastWakeTime;	  
	portTickType	xLastWakeTimeChk;	 
	uint8_t 		i;

	/* init code for FATFS */
	MX_FATFS_Init();

	//�ð� �ʱ�ȭ 
	HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
	
	if( (SysTime.Date.Year < 10)  || (SysTime.Date.Year > 99)	 || (SysTime.Date.Month > 12)	||	(SysTime.Date.Date > 31) || 
		(SysTime.Time.Hours > 23) || (SysTime.Time.Minutes > 59) || (SysTime.Time.Seconds > 59) )
	{
		SysTime.Date.Year	 = 19;
		SysTime.Date.Month	 = 1;
		SysTime.Date.Date	 = 1;
		SysTime.Date.WeekDay = RTC_WEEKDAY_TUESDAY;
		SysTime.Time.Hours	 = 0;
		SysTime.Time.Minutes = 0;
		SysTime.Time.Seconds = 0;
	
		HAL_RTC_SetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
	}

	//Task ���� �Ϸ� �÷��� 
	SysProperties.bootingWate[3] = TRUE;

	while(1)
	{
		if( (SysProperties.bootingWate[0] == TRUE) &&	// 0 : StartDiaplayTask,
			(SysProperties.bootingWate[1] == TRUE) &&	// 1 : StartRs485Task, 
			(SysProperties.bootingWate[2] == TRUE) &&	// 2 : StartSlotUartTask, 
			(SysProperties.bootingWate[3] == TRUE) )	// 3 : StartRateTask
		{
			break;
		}
		osDelay(100);
	}

	if(sdValue.sdMountState == SCS_OK)		//sd card Link Ȯ�� 
	{
		if(MountSDIO() != FR_OK)			//sd card mount Ȯ�� 
		{
			sdValue.sdMountState = SCS_MOUNT_ERROR;
			doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32);
			SendUart485String(tx485DataDMA, 32);
		}
		else
		{
			sdValue.sdMountState = SCS_OK;
		}
	}
	else
	{
		sdValue.sdState = SCS_MOUNT_ERROR;
		doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32);
		SendUart485String(tx485DataDMA, 32);
	}

	/* Infinite loop */
	for(;;)
	{
		xLastWakeTime = osKernelSysTick();

		if(!FindFilelistFlag)	//���� ����Ʈ �˻��� �� ���� ������ ���� 
			{
			if(SysProperties.InterfaceStep == STEP_TEMP_READ)
			{
				if(sdValue.sdMountState == SCS_OK)	// Mount ���� ���� �������� �õ���. 
				{
					DoSdCardFunction();
					DoSdCardFreeSpace();			// sd card ���� ���� ���� �߻� Ȯ�� 
				}
				else
				{
					FATFS_UnLinkDriver((char*)SDPath);
					MX_FATFS_Init();
				}			
			}
		}
		
		if(SysProperties.start_flag == TRUE)
		{
			DoMCUboardInfo();		//mcu board ���� ���� 
			osDelay(10);

			for(i = 0; i < 4; i++)
			{
				DoSlotInfo(i);		//���� ���� ���� 
				osDelay(10);
				DoChannelInfo(i);	//ä�� ���� ����
				osDelay(10);
				DoChannelValue(i);	//ä�� �µ� ���� 
				osDelay(10);

			}
		}

		DoSmpsCheck();
		
		xLastWakeTimeChk = osKernelSysTick();
		if(xLastWakeTimeChk - xLastWakeTime >= SysProperties.intervalTime.UI32)
			osDelay(10);
		else
			osDelayUntil(&xLastWakeTime, (uint32_t)SysProperties.intervalTime.UI32);
	}
	/* USER CODE END 5 */ 
}

void DoSmpsCheck(void)
{
	SysProperties.smpsState = 0;
	if(HAL_GPIO_ReadPin(SMPS1_DECTECT_GPIO_Port, SMPS1_DECTECT_Pin) == GPIO_PIN_RESET)
	{
		SysProperties.smpsState |= 0x01;
	}
	
	if(HAL_GPIO_ReadPin(SMPS2_DECTECT_GPIO_Port, SMPS2_DECTECT_Pin) == GPIO_PIN_RESET)
	{
		SysProperties.smpsState |= 0x02;
	}
}

void DoSdCardFreeSpace(void)
{
	uint64_t all = 0;
	uint64_t free = 0;
	uint64_t ss = 0;
	float    persent = 0.0f;

	ss   = (uint64_t)_MAX_SS;	
	all  = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.n_fatent);  	// ��ü�뷮 
    free = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.free_clst);	// ���� ����
	persent = (float)((float)free / (float)all);

	if(persent < SD_CARD_FULL_ERROR_RATE)
	{
		sdValue.sdState = SCS_DISK_FULL;
	    doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, &sdValue.sdState, 1, 12, 32);
	    SendUart485String(tx485DataDMA, 32);	
	}
	else
	{
		sdValue.sdState = SCS_OK;
	}
}

void DoSdCardFunction(void)
{
    HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
	
	if(DoFolderCheck() == FR_OK)		// ���丮 ���� �� Ȯ�� 
	{
		sdValue.sdState = SCS_OK;
		DoFileCheck();		// ���� ���¸� Ȯ���ϰ� ���� �ش� ���� ���� �Ѵ�. , ������ ������� ���� ���Ϸ� �̾ ����. 
		DoDataWrite();
	}
	else
	{
		sdValue.sdState = SCS_MKDIR_ERROR;
		tempData[0] = (uint8_t)sdValue.sdState;
	    doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, tempData, 1, 12, 32);
	    SendUart485String(tx485DataDMA, 32);		
	}	
}

void DoMCUboardInfo(void)
{
	RealTimeSendData[0] = 0;		//�θ� ID (MCU Board ID�� 0���� �ϴ� ����)
	util_mem_cpy(&RealTimeSendData[1], &TestData.mainBoard[MBS_BATTERY].UI8[0], 16);	
	RealTimeSendData[17] = sdValue.sdState;
		
	doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_MAIN_INFO, &RealTimeSendData[0], 17, 32, 52);	
	SendUart485String(tx485DataDMA, 52);
}

void DoSlotInfo(uint8_t slot)
{
	RealTimeSendData[0] = 0;		//�θ� ID (MCU Board ID�� 0���� �ϴ� ����)
	RealTimeSendData[1] = slot;
	RealTimeSendData[2] = SBT_NTC;	//todo : slot�� ������ �������� ���� �о ��� �ؾ� �� 
	RealTimeSendData[3] = TestData.revisionApply[slot];

	doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_SLOT_INFO, &RealTimeSendData[0], 4, 32, 52);	
	SendUart485String(tx485DataDMA, 52);
}

void DoChannelInfo(uint8_t slot)
{
	RealTimeSendData[0] = slot;
	util_mem_cpy(&RealTimeSendData[1], &TestData.sensorState[slot][0], 32);	
	
	doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_CHANNEL_INFO, &RealTimeSendData[0], 33, 36, 56);	
	SendUart485String(tx485DataDMA, 56);
}

void DoChannelValue(uint8_t slot)
{
	RealTimeSendData[0] = slot;
	util_mem_cpy(&RealTimeSendData[1], &TestData.temperature[slot][0].UI8[0], 128);	

	doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_CHANNEL_VALUE, &RealTimeSendData[0], 129, 132, 152); 
	SendUart485String(tx485DataDMA, 152);
}

