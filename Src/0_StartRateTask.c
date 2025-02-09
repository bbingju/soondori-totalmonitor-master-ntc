#include "0_StartRateTask.h"
#include "0_GlobalValue.h"
#include "external_uart_task.h"
#include "job_task.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "fatfs.h"
#include "0_SdCard.h"

#include <stdio.h>

/*********************************************************************
*	Private variables
**********************************************************************/
uint8_t         RealTimeSendData[160] = { 0 };
uint32_t        nameCount = 0;
uint8_t         tempData[1];

/* SYSTEM_TIME     SysTime; */
/* SD_CARD_VALUE	sdValue; */

extern Disk_drvTypeDef disk;

extern IWDG_HandleTypeDef hiwdg;
/*********************************************************************
*	StartDisplayTask
*	16-SEGMENT, LED, BUTTON INPUT 등을 처리 하는 TASK
**********************************************************************/
void StartRateTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	portTickType	xLastWakeTime;
	portTickType	xLastWakeTimeChk;

	/* init code for FATFS */

	RTC_DateTypeDef *date = &SysTime.Date;
	RTC_TimeTypeDef *time = &SysTime.Time;

	//시간 초기화
	HAL_RTC_GetDate(&hrtc, date, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN);

	if( (date->Year < 10) || (date->Year > 99) || (date->Month > 12) || (date->Date > 31) ||
		(time->Hours > 23) || (time->Minutes > 59) || (time->Seconds > 59) ) {
		date->Year    = 19;
		date->Month   = 1;
		date->Date    = 1;
		date->WeekDay = RTC_WEEKDAY_TUESDAY;
		time->Hours   = 0;
		time->Minutes = 0;
		time->Seconds = 0;

		HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
	}

	/* if (sdValue.sdMountState == SCS_OK) { //sd card Link 확인 */
	/* 	if (MountSDIO() != FR_OK) { //sd card mount 확인 */
	/* 		sdValue.sdMountState = SCS_MOUNT_ERROR; */
	/* 		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32); */
	/* 	} */
	/* 	else { */
	/* 		sdValue.sdMountState = SCS_OK; */
	/* 	} */
	/* } */
	/* else { */
	/* 	ctx.sd_last_error = SCS_MOUNT_ERROR; */
	/* 	send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32); */
	/* } */

	xLastWakeTime = osKernelSysTick();
	/* Infinite loop */
	for (;;) {
		/* xLastWakeTime = osKernelSysTick(); */

		/* if (!FindFilelistFlag) //파일 리스트 검색중 일 때 쓰기 안함 */
		/* { */
		/* 	if(SysProperties.InterfaceStep == STEP_TEMP_READ) */
		/* 	{ */
		/* 		if(sdValue.sdMountState == SCS_OK) // Mount 까지 성공 했을때만 시도함. */
		/* 		{ */
		/* 			DoSdCardFunction(); */
		/* 			DoSdCardFreeSpace(); // sd card 공간 부족 에러 발생 확인 */
		/* 		} */
		/* 		else */
		/* 		{ */
		/* 			FATFS_UnLinkDriver((char*)SDPath); */
		/* 			MX_FATFS_Init(); */
		/* 		} */
		/* 	} */
		/* } */

		if (SysProperties.start_flag == TRUE) {
			DoMCUboardInfo(); // transmit board info
			osDelay(10);

			for (int i = 0; i < MAX_SLOT_NUM; i++) {
				DoSlotInfo(i);     // transmit slot info
				osDelay(10);
				DoChannelInfo(i);  // trinsmit channel state info
				osDelay(10);
				DoChannelValue(i); // transmit temperature values
				osDelay(10);

			}
		}

		DoSmpsCheck();

		/* xLastWakeTimeChk = osKernelSysTick(); */
		/* if (xLastWakeTimeChk - xLastWakeTime >= SysProperties.interval_ms) */
		/* 	osDelay(10); */
		/* else */
			osDelayUntil(&xLastWakeTime, (uint32_t)SysProperties.interval_ms);
	}
}

void DoSdCardFreeSpace(void)
{
	uint64_t all = 0;
	uint64_t free = 0;
	uint64_t ss = 0;
	float    persent = 0.0f;

	ss   = (uint64_t)_MAX_SS;
	/* all  = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.n_fatent);     // 전체용량 */
	/* free = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.free_clst);	// 남은 공간 */
	all  = (uint64_t)(ss * (uint64_t)SDFatFS.n_fatent); // 전체용량
	free = (uint64_t)(ss * (uint64_t)SDFatFS.free_clst); // 남은 공간
	persent = (float)((float)free / (float)all);

	if (persent < SD_CARD_FULL_ERROR_RATE) {
		ctx.sd_last_error = SCS_DISK_FULL;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
	} else {
		ctx.sd_last_error = SCS_OK;
	}
}

void DoSdCardFunction(void)
{
	HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);

	if (DoFolderCheck() == FR_OK)		// 디렉토리 생성 및 확인
	{
		ctx.sd_last_error = SCS_OK;
		DoFileCheck();		// 파일 상태를 확인하고 파일 해더 생성 까지 한다. , 파일이 있을경우 기존 파일로 이어서 쓴다.
		DoDataWrite();
	}
	else
	{
		ctx.sd_last_error = SCS_MKDIR_ERROR;
		tempData[0] = (uint8_t)ctx.sd_last_error;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, tempData, 1, 12, 32);
	}
}
