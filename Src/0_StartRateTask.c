#include "0_StartRateTask.h"
#include "0_GlobalValue.h"
#include "external_uart_task.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "fatfs.h"
#include "0_SdCard.h"

#include <stdio.h>              /* sprintf() */

/*********************************************************************
*	Private variables
**********************************************************************/
uint8_t         RealTimeSendData[160] = { 0 };
uint32_t        nameCount = 0;
uint8_t         tempData[1];

SYSTEM_TIME     SysTime;
SD_CARD_VALUE	sdValue;

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
    uint8_t             i;

    /* init code for FATFS */
    MX_FATFS_Init();
    HAL_IWDG_Refresh(&hiwdg);

    //시간 초기화
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

    /* //Task 부팅 완료 플레그 */
    /* SysProperties.bootingWate[3] = TRUE; */

    /* while(1) */
    /* { */
    /*     if( (SysProperties.bootingWate[0] == TRUE) &&	// 0 : StartDiaplayTask, */
    /*         (SysProperties.bootingWate[1] == TRUE) &&	// 1 : StartRs485Task, */
    /*         (SysProperties.bootingWate[2] == TRUE) &&	// 2 : StartSlotUartTask, */
    /*         (SysProperties.bootingWate[3] == TRUE) )	// 3 : StartRateTask */
    /*     { */
    /*         break; */
    /*     } */
    /*     osDelay(100); */
    /* } */

    if(sdValue.sdMountState == SCS_OK)		//sd card Link 확인
    {
        if(MountSDIO() != FR_OK)			//sd card mount 확인
        {
            sdValue.sdMountState = SCS_MOUNT_ERROR;
            send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32);
            /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32); */
            /* SendUart485String(tx485DataDMA, 32); */
        }
        else
        {
            sdValue.sdMountState = SCS_OK;
        }
    }
    else
    {
        sdValue.sdState = SCS_MOUNT_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, (uint8_t*)sdValue.sdState, 1, 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
    }

    /* Infinite loop */
    for(;;)
    {
        xLastWakeTime = osKernelSysTick();

        if(!FindFilelistFlag)	//파일 리스트 검색중 일 ㄷ대 ㅅ스기 안함
        {
            if(SysProperties.InterfaceStep == STEP_TEMP_READ)
            {
                if(sdValue.sdMountState == SCS_OK)	// Mount 까지 성공 했을때만 시도함.
                {
                    DoSdCardFunction();
                    DoSdCardFreeSpace();			// sd card 공간 부족 에러 발생 확인
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
            DoMCUboardInfo();		//mcu board 정보 전송
            osDelay(10);

            for(i = 0; i < 4; i++)
            {
                DoSlotInfo(i);		//슬롯 정보 전송
                osDelay(10);
                DoChannelInfo(i);	//채널 정보 전송
                osDelay(10);
                DoChannelValue(i);	//채널 온도 전송
                osDelay(10);

            }
        }

        DoSmpsCheck();

        xLastWakeTimeChk = osKernelSysTick();
        if (xLastWakeTimeChk - xLastWakeTime >= SysProperties.interval_ms)
            osDelay(10);
        else
            osDelayUntil(&xLastWakeTime, (uint32_t)SysProperties.interval_ms);
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
    all  = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.n_fatent);     // 전체용량
    free = (uint64_t)(ss * (uint64_t)sdValue.sdFatFs.free_clst);	// 남은 공간
    persent = (float)((float)free / (float)all);

    if(persent < SD_CARD_FULL_ERROR_RATE)
    {
        sdValue.sdState = SCS_DISK_FULL;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, &sdValue.sdState, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, &sdValue.sdState, 1, 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
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

    if(DoFolderCheck() == FR_OK)		// 디렉토리 생성 및 확인
    {
        sdValue.sdState = SCS_OK;
        DoFileCheck();		// 파일 상태를 확인하고 파일 해더 생성 까지 한다. , 파일이 있을경우 기존 파일로 이어서 쓴다.
        DoDataWrite();
    }
    else
    {
        sdValue.sdState = SCS_MKDIR_ERROR;
        tempData[0] = (uint8_t)sdValue.sdState;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, tempData, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, tempData, 1, 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
    }
}

void DoMCUboardInfo(void)
{
    RealTimeSendData[0] = 0;		//부모 ID (MCU Board ID는 0으로 일단 고정)
    memcpy(&RealTimeSendData[1], &TestData.mainBoard[MBS_BATTERY].UI8[0], 16);
    RealTimeSendData[17] = sdValue.sdState;

    send_external_response(CMD_TEMP_TEST, OP_TEMP_MAIN_INFO, &RealTimeSendData[0], 17, 32, 52);
    /* doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_MAIN_INFO, &RealTimeSendData[0], 17, 32, 52); */
    /* SendUart485String(tx485DataDMA, 52); */
}

void DoSlotInfo(uint8_t slot)
{
    RealTimeSendData[0] = 0;		//부모 ID (MCU Board ID는 0으로 일단 고정)
    RealTimeSendData[1] = slot;
    RealTimeSendData[2] = SBT_NTC;	//todo : slot의 종류가 많아지면 정보 읽어서 기록 해야 함
    RealTimeSendData[3] = TestData.revisionApply[slot];

    send_external_response(CMD_TEMP_TEST, OP_TEMP_SLOT_INFO, &RealTimeSendData[0], 4, 32, 52);
    /* doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_SLOT_INFO, &RealTimeSendData[0], 4, 32, 52); */
    /* SendUart485String(tx485DataDMA, 52); */
}

void DoChannelInfo(uint8_t slot)
{
    RealTimeSendData[0] = slot;
    memcpy(&RealTimeSendData[1], &TestData.sensorState[slot][0], 32);

    send_external_response(CMD_TEMP_TEST, OP_TEMP_CHANNEL_INFO, &RealTimeSendData[0], 33, 36, 56);
    /* doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_CHANNEL_INFO, &RealTimeSendData[0], 33, 36, 56); */
    /* SendUart485String(tx485DataDMA, 56); */
}

void DoChannelValue(uint8_t slot)
{
    RealTimeSendData[0] = slot;
    memcpy(&RealTimeSendData[1], &TestData.temperature[slot][0].UI8[0], 128);

    send_external_response(CMD_TEMP_TEST, OP_TEMP_CHANNEL_VALUE, &RealTimeSendData[0], 129, 132, 152);
    /* doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_CHANNEL_VALUE, &RealTimeSendData[0], 129, 132, 152); */
    /* SendUart485String(tx485DataDMA, 152); */
}
