#include "0_StartDisplayTask.h"
#include "0_16Segment.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"

/*********************************************************************
*	Private variables
**********************************************************************/
TEST_DATA TestData;

uint8_t		ct = 0;
uint32_t	adc_battery[110];
uint32_t	adc_mainBoardSensor[310];
uint32_t	adc_battert_add = 0;
uint32_t	adc_mainBoardRTD_add = 0;
float 		adc_mainBoardTEMP_add = 0;
float 		adc_mainBoardHUMI_add = 0;

BUTTONMODE	modeButtonEnter = BTN_NORMAL;
uint8_t		modeButtonDelay;

SEGMENT_SPECIAL_FONT 	digit_L = SSP_HYPHEN;
SEGMENT_SPECIAL_FONT	digit_R = SSP_HYPHEN;

uint8_t 	tempErrorChennal = 0;

uint32_t 	revid;
uint32_t 	devid;
uint32_t 	uid[3];

uint8_t iiii;

/*********************************************************************
*	StartDisplayTask
*	16-SEGMENT, LED, BUTTON INPUT ���� ó�� �ϴ� TASK
**********************************************************************/
void StartDisplayTask(void const * argument)
{
	portTickType	  xLastWakeTime;

	/* USER CODE BEGIN 5 */
    osDelay(500);

	revid = HAL_GetREVID();
	devid = HAL_GetDEVID();

    //Task ���� �Ϸ� �÷���
    SysProperties.bootingWate[0] = TRUE;

    while(1)
    {
        if( (SysProperties.bootingWate[0] == TRUE) &&   // 0 : StartDiaplayTask,
            (SysProperties.bootingWate[1] == TRUE) &&   // 1 : StartRs485Task,
            (SysProperties.bootingWate[2] == TRUE) &&   // 2 : StartSlotUartTask,
            (SysProperties.bootingWate[3] == TRUE) )    // 3 : StartRateTask
        {
            break;
        }
        osDelay(100);
    }

    /* Infinite loop */
    for(;;)
    {
        xLastWakeTime = osKernelSysTick();

        if(ct++ % 2 == 0)	// 1�ʿ� �ѹ��� ����
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
*	16Segment ǥ�� ��� ������ ���� ���� üũ
**********************************************************************/
void DoDisplayModeChange(void)
{
	uint8_t	i, j;
	uint8_t errorCount = 0;

	//�µ� ��� �ִ��� Ȯ��
	for(j = 0; j < 4; j++)
	{
		for(i = 0; i < 16; i++)
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

	//sd ī�� ���� Ȯ��
	if(sdValue.sdState != SCS_OK)
	{
		SysProperties.displayMode = DPM_SDCARD_ERROR;
		return;
	}

	//���� ����
	SysProperties.displayMode = DPM_NORMAL;
}

/*********************************************************************
*	doSegmentDisplay
*	ǥ�� ��庰 16Segment ǥ��
**********************************************************************/
void doSegmentDisplay(uint8_t quarterSec)
{
	switch(SysProperties.displayMode){
		case DPM_NORMAL:
			HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, GPIO_PIN_SET);

			SegmentDisplay(1, doNumberToDigitdata(digit_L));
			SegmentDisplay(2, doNumberToDigitdata(digit_R));

			digit_L++;
			digit_R++;

			if(digit_L > SSP_SLASH)
			{
				digit_L = SSP_HYPHEN;
				digit_R = SSP_HYPHEN;
			}
			break;
		case DPM_TEMP_ERROR:
			HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, GPIO_PIN_RESET);
			doBuzzerPlay(200);

			if((quarterSec % 4) < 2)
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
*	��� ��ư ó��
**********************************************************************/
void doModeButton(void)
{
	//Mode Button
	if(myBinarySemModeHandle != NULL)
	{
		if(osSemaphoreWait(myBinarySemModeHandle, 0) == osOK)	// ��ư ���ȴ��� Ȯ��
		{
			modeButtonEnter = BTN_FALLING;		// ��ư ����
		}
	}

	if(HAL_GPIO_ReadPin(MODE_BUTTON_GPIO_Port, MODE_BUTTON_Pin) == GPIO_PIN_SET)	//��ư���� �� ���� ����
	{
		modeButtonEnter = BTN_NORMAL;
		modeButtonDelay = 0;
	}

	if(modeButtonEnter == BTN_FALLING)		//��ư ������ Ȯ�Ή�����
	{
		modeButtonDelay++;
		if(modeButtonDelay > 12)						//��ư ������ 3�� Ȯ��
		{
			if(SysProperties.displayMode == DPM_NORMAL)
			{
				modeButtonEnter = BTN_ENTERED;		//��ư 3�� �̻� ������, ������ ����
				doBuzzerPlay(90);
				osDelay(40);
				doBuzzerPlay(90);
				SysProperties.displayMode = DPM_SETTING;
			}
			else if(SysProperties.displayMode == DPM_SETTING)
			{
				modeButtonEnter = BTN_ENTERED;		//��ư 3�� �̻� ������, ������ ����
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
*	���͸� ���� Ȯ��
**********************************************************************/
void doBatteryVoltageCheck(void)
{
	uint8_t i;

	adc_battert_add = 0;
	for(i = 10; i < 100; i++){	//dma �� �Ϥ�� �������� ó�� ��� ���� ������ ���ܼ� 10���� �ǳʤ��ھ �Ի� ��.
		adc_battert_add += adc_battery[i];
	}
	TestData.mainBoard[MBS_BATTERY].Float = (float)(((double)( ( (double) ((double)adc_battert_add
																					/ 90) 						// 90 : �հԵ� ����
																					* 2)						// 2 : ���ο��� 1/2�� ������
																					* 3300						// 3.3V * 1000
																					/ 0xfff)					// 4095 resolution
																					/ 1000) 					// 3.3V �� ȯ��
																					+ 0.1);						// diode drop voltage ����

	HAL_ADC_Start_DMA(&hadc1, adc_battery, 100);
}

/*********************************************************************
*	doMainBoardSensorCheck
*	���κ��� ���� ���� Ȯ��
**********************************************************************/
void doMainBoardSensorCheck(void)
{
	uint8_t i;
  //float returnedValue = 0;

	adc_mainBoardRTD_add = 0;
	adc_mainBoardTEMP_add = 0;
	adc_mainBoardHUMI_add = 0;

	// RTD DATA
	for(i = 0; i < 100; i++)
	{
	    adc_mainBoardRTD_add += adc_mainBoardSensor[i * 3];
	}
	TestData.mainBoardADC[MBS_RTD] = adc_mainBoardRTD_add / 100;
	TestData.mainBoard[MBS_RTD].Float = Calc_Temp_RTD(TestData.mainBoardADC[MBS_RTD]) + TestData.rtdCalibrationConst.Float;	//���� ��� ��

	// BOARD TEMP
	for(i = 0; i < 100; i++){
		adc_mainBoardTEMP_add += Calc_BD_Temp(adc_mainBoardSensor[i * 3 + 1]);
	}
	TestData.mainBoard[MBS_TEMP].Float = adc_mainBoardTEMP_add / 100;

	// BOARD HUMIDITY
	for(i = 0; i < 100; i++){
		adc_mainBoardHUMI_add += Calc_BD_Humi(adc_mainBoardSensor[i * 3 + 2]);
	}
	TestData.mainBoard[MBS_HUMI].Float = adc_mainBoardHUMI_add / 100;

	HAL_ADC_Start_DMA(&hadc2, adc_mainBoardSensor, 300);
}
