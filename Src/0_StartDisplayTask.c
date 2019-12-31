#include "0_StartDisplayTask.h"
#include "0_16Segment.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"

/*********************************************************************
*	Private variables
**********************************************************************/
TEST_DATA TestData;

uint32_t	adc_battery[110];
uint32_t	adc_mainBoardSensor[310];
uint32_t	adc_battert_add = 0;
uint32_t	adc_mainBoardRTD_add = 0;
float		adc_mainBoardTEMP_add = 0;
float		adc_mainBoardHUMI_add = 0;

BUTTONMODE	modeButtonEnter = BTN_NORMAL;
uint8_t		modeButtonDelay;

SEGMENT_SPECIAL_FONT	digit_L = SSP_HYPHEN;
SEGMENT_SPECIAL_FONT	digit_R = SSP_HYPHEN;

uint8_t		tempErrorChennal = 0;

uint32_t	revid;
uint32_t	devid;
uint32_t	uid[3];

uint8_t iiii;

extern IWDG_HandleTypeDef hiwdg;

/*********************************************************************
*	StartDisplayTask
*	16-SEGMENT, LED, BUTTON INPUT 등을 처리 하는 TASK
**********************************************************************/
void StartDisplayTask(void const * argument)
{
    portTickType	  xLastWakeTime;

    HAL_IWDG_Refresh(&hiwdg);
    osDelay(500);

    revid = HAL_GetREVID();
    devid = HAL_GetDEVID();

    //Task 부팅 완료 플레그
    /* SysProperties.bootingWate[0] = TRUE; */

    /* while(1) */
    /* { */
    /*     if( (SysProperties.bootingWate[0] == TRUE) &&   // 0 : StartDiaplayTask, */
    /*         (SysProperties.bootingWate[1] == TRUE) &&   // 1 : StartRs485Task, */
    /*         (SysProperties.bootingWate[2] == TRUE) &&   // 2 : StartSlotUartTask, */
    /*         (SysProperties.bootingWate[3] == TRUE) )    // 3 : StartRateTask */
    /*     { */
    /*         break; */
    /*     } */
    /*     osDelay(100); */
    /* } */

    /* Infinite loop */
    for(;;)
    {
	static uint8_t ct = 0;
	xLastWakeTime = osKernelSysTick();

	if(ct++ % 2 == 0)	// 1초에 한번씩 점멸
	{
	    HAL_GPIO_TogglePin(POWER_LED_GPIO_Port, POWER_LED_Pin);
	    DoDisplayModeChange();
	    doSegmentDisplay(ct);
	}

	doBatteryVoltageCheck();
	doMainBoardSensorCheck();
	doModeButton();

	//Up Button
	if(myBinarySemUpHandle != NULL)
	{
	    //if(osSemaphoreWait(myBinarySemUpHandle, 100) == osOK)
	    {
		HAL_GPIO_TogglePin(SD_LED_GPIO_Port, SD_LED_Pin);
	    }
	}

	//Down Button
	if(myBinarySemDownHandle != NULL)
	{
	    //if(osSemaphoreWait(myBinarySemDownHandle, 100) == osOK)
	    {
		HAL_GPIO_TogglePin(SD_LED_GPIO_Port, SD_LED_Pin);
	    }
	}
	osDelayUntil(&xLastWakeTime, 250);
    }
    /* USER CODE END 5 */
}

/*********************************************************************
*	DoDisplayModeChange
*	16Segment 표시 모드 변경을 위해 에러 체크
**********************************************************************/
void DoDisplayModeChange(void)
{
	uint8_t errorCount = 0;

	//온도 경고가 있는지 확인
	for(int j = 0; j < 4; j++)
	{
		for(int i = 0; i < 16; i++)
		{
			if(TestData.sensorState[j][i] == LDM_OVER_TEMP)
			{
				SysProperties.displayMode = DPM_TEMP_ERROR;
				tempErrorChennal = ((j * 16) + i) + 1;
				errorCount++;
				return;
			}
		}
	}

	//sd 카드 에러 확인
	if(sdValue.sdState != SCS_OK)
	{
		SysProperties.displayMode = DPM_SDCARD_ERROR;
		return;
	}

	//에러 없음
	SysProperties.displayMode = DPM_NORMAL;
}

/*********************************************************************
*	doSegmentDisplay
*	표시 모드별 16Segment 표시
**********************************************************************/
void doSegmentDisplay(uint8_t quarterSec)
{
	switch (SysProperties.displayMode) {
		case DPM_NORMAL:
			HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, GPIO_PIN_SET);

			SegmentDisplay(1, doNumberToDigitdata(digit_L));
			SegmentDisplay(2, doNumberToDigitdata(digit_R));

			digit_L++;
			digit_R++;

			if (digit_L > SSP_SLASH)
			{
				digit_L = SSP_HYPHEN;
				digit_R = SSP_HYPHEN;
			}
			break;
		case DPM_TEMP_ERROR:
			HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, GPIO_PIN_RESET);
			doBuzzerPlay(200);

			if ((quarterSec % 4) < 2)
			{
				SegmentDisplay(1, doTextToDigitdata('E'));
				SegmentDisplay(2, doTextToDigitdata('T'));
			}
			else
			{
				SegmentDisplay(1, doTextToDigitdata((tempErrorChennal / 10) + '0'));
				SegmentDisplay(2, doTextToDigitdata((tempErrorChennal % 10) + '0'));
			}
			break;
		case DPM_SDCARD_ERROR:
/*
			if((quarterSec % 4) < 2)
			{
				//doBuzzerPlay(100);
				SegmentDisplay(1, doTextToDigitdata('E'));
				SegmentDisplay(2, doTextToDigitdata('S'));
			}
			else
			{
				//iiii = (uint8_t)sdValue.sdState;
				SegmentDisplay(1, doTextToDigitdata((uint8_t)(sdValue.sdState) / (uint8_t)10 + '0'));
				SegmentDisplay(2, doTextToDigitdata((uint8_t)(sdValue.sdState) % (uint8_t)10 + '0'));
			}*/
			break;
		case DPM_SETTING:
			SegmentDisplay(1, doTextToDigitdata('S'));
			SegmentDisplay(2, doTextToDigitdata('E'));
			break;
	}
}

/*********************************************************************
*	doModeButton
*	모드 버튼 처리
**********************************************************************/
void doModeButton(void)
{
	//Mode Button
	if(myBinarySemModeHandle != NULL)
	{
		if(osSemaphoreWait(myBinarySemModeHandle, 0) == osOK) // 버튼 눌렸는지 확인
		{
			modeButtonEnter = BTN_FALLING; // 버튼 눌림
		}
	}

	if(HAL_GPIO_ReadPin(MODE_BUTTON_GPIO_Port, MODE_BUTTON_Pin) == GPIO_PIN_SET) //버튼에서 손 때면 복귀
	{
		modeButtonEnter = BTN_NORMAL;
		modeButtonDelay = 0;
	}

	if(modeButtonEnter == BTN_FALLING) //버튼 눌린거 확인됬을때
	{
		modeButtonDelay++;
		if(modeButtonDelay > 12) //버튼 누르고 3초 확인
		{
			if(SysProperties.displayMode == DPM_NORMAL)
			{
				modeButtonEnter = BTN_ENTERED; //버튼 3초 이상 눌렸음, 제진입 금지
				doBuzzerPlay(90);
				osDelay(40);
				doBuzzerPlay(90);
				SysProperties.displayMode = DPM_SETTING;
			}
			else if(SysProperties.displayMode == DPM_SETTING)
			{
				modeButtonEnter = BTN_ENTERED; //버튼 3초 이상 눌렸음, 제진입 금지
				doBuzzerPlay(90);
				osDelay(40);
				doBuzzerPlay(90);
				SysProperties.displayMode = DPM_NORMAL;
			}
		}
	}
}

/*********************************************************************
*	doBatteryVoltageCheck
*	배터리 전원 확인
**********************************************************************/
void doBatteryVoltageCheck(void)
{
	adc_battert_add = 0;
	for (int i = 10; i < 100; i++){	//dma 로 일ㄱ어낸 데이터중 처름 몇개의 값이 오차가 생겨서 10개는 건너ㄷ뒤어서 게산 함.
		adc_battert_add += adc_battery[i];
	}
	TestData.mainBoard[MBS_BATTERY].Float = (float)(((double)( ( (double) ((double)adc_battert_add
										/ 90) /* 90 : 합게된 수량 */
										* 2) /* 2 : 내부에서 1/2로 배율됨 */
										* 3300 /* 3.3V * 1000 */
										/ 0xfff) /* 4095 resolution */
										/ 1000) /* 3.3V 로 환산 */
										+ 0.1);	/* diode drop voltage 보상 */

	HAL_ADC_Start_DMA(&hadc1, adc_battery, 100);
}

/*********************************************************************
*	doMainBoardSensorCheck
*	메인보드 내부 센서 확인
**********************************************************************/
void doMainBoardSensorCheck(void)
{
	adc_mainBoardRTD_add = 0;
	adc_mainBoardTEMP_add = 0;
	adc_mainBoardHUMI_add = 0;

	// RTD DATA
	for (int i = 0; i < 100; i++)
	{
	    adc_mainBoardRTD_add += adc_mainBoardSensor[i * 3];
	}
	TestData.mainBoardADC[MBS_RTD] = adc_mainBoardRTD_add / 100;
	TestData.mainBoard[MBS_RTD].Float = Calc_Temp_RTD(TestData.mainBoardADC[MBS_RTD]) + TestData.rtdCalibrationConst.Float;	//교정 상수 추

	// BOARD TEMP
	for (int i = 0; i < 100; i++) {
		adc_mainBoardTEMP_add += Calc_BD_Temp(adc_mainBoardSensor[i * 3 + 1]);
	}
	TestData.mainBoard[MBS_TEMP].Float = adc_mainBoardTEMP_add / 100;

	// BOARD HUMIDITY
	for (int i = 0; i < 100; i++) {
		adc_mainBoardHUMI_add += Calc_BD_Humi(adc_mainBoardSensor[i * 3 + 2]);
	}
	TestData.mainBoard[MBS_HUMI].Float = adc_mainBoardHUMI_add / 100;

	HAL_ADC_Start_DMA(&hadc2, adc_mainBoardSensor, 300);
}
