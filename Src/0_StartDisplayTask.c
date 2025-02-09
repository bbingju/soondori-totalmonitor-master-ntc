#include "0_StartDisplayTask.h"
#include "app_ctx.h"
#include "0_16Segment.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"
#include "debug.h"

#include <string.h>

/*********************************************************************
*	Private variables
**********************************************************************/
TEST_DATA TestData;

#define SAMPLE_NBR 100

static uint32_t adc_battery[SAMPLE_NBR + 10];
uint32_t	adc_mainBoardSensor[SAMPLE_NBR * 3];

BUTTONMODE	modeButtonEnter = BTN_NORMAL;
uint8_t		modeButtonDelay;

SEGMENT_SPECIAL_FONT	digit_L = SSP_HYPHEN;
SEGMENT_SPECIAL_FONT	digit_R = SSP_HYPHEN;

uint8_t		tempErrorChannel = 0;

uint32_t	revid;
uint32_t	devid;
uint32_t	uid[3];


extern app_ctx_t ctx;
extern IWDG_HandleTypeDef hiwdg;

static uint32_t get_middle_value(uint32_t * array, int nbr);

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

	/* Infinite loop */
	for(;;)
	{
		static uint8_t ct = 0;
		xLastWakeTime = osKernelSysTick();

		if (ct++ % 2 == 0)	// 1초에 한번씩 점멸
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
	FOREACH(struct slot_s *s, ctx.slots) {
		if (!s->inserted) continue;
		for (int i = 0; i < 16; i++) {
			if (s->ntc.channel_states[i] == CHANNEL_STATE_OVER_TEMP) {
				SysProperties.displayMode = DPM_TEMP_ERROR;
				tempErrorChannel = ((s->id * 16) + i) + 1;
				errorCount++;
				return;
			}
		}
	}

	//sd 카드 에러 확인
	if (ctx.sd_last_error != SD_RET_OK) {
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
			SegmentDisplay(1, doTextToDigitdata((tempErrorChannel / 10) + '0'));
			SegmentDisplay(2, doTextToDigitdata((tempErrorChannel % 10) + '0'));
		}
		break;
	case DPM_SDCARD_ERROR:
		if ((quarterSec % 4) < 2)
		{
			SegmentDisplay(1, doTextToDigitdata('E'));
			SegmentDisplay(2, doTextToDigitdata('S'));
		}
		else
		{
			SegmentDisplay(1, doTextToDigitdata((uint8_t)(ctx.sd_last_error) / (uint8_t)10 + '0'));
			SegmentDisplay(2, doTextToDigitdata((uint8_t)(ctx.sd_last_error) % (uint8_t)10 + '0'));
		}
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
	uint32_t adc_battery_add = 0;

	/* dma 로 일ㄱ어낸 데이터중 처음 몇개의 값이 오차가 생겨서
	 * 10개는 건너ㄷ뒤어서 게산 함. */
	for (int i = 10; i < SAMPLE_NBR; i++){
		adc_battery_add += adc_battery[i];
	}

	ctx.battery = (float)(((double)( ( (double) ((double)adc_battery_add/ 90) /* 90 : 합게된 수량 */
					* 2) /* 2 : 내부에서 1/2로 배율됨 */
					* 3300 /* 3.3V * 1000 */
					/ 0xfff) /* 4095 resolution */
					/ 1000) /* 3.3V 로 환산 */
					+ 0.1);	/* diode drop voltage 보상 */
	/* TestData.mainBoard[MBS_BATTERY].Float = (float)(((double)( ( (double) ((double)adc_battery_add */
	/* 									/ 90) /\* 90 : 합게된 수량 *\/ */
	/* 									* 2) /\* 2 : 내부에서 1/2로 배율됨 *\/ */
	/* 									* 3300 /\* 3.3V * 1000 *\/ */
	/* 									/ 0xfff) /\* 4095 resolution *\/ */
	/* 									/ 1000) /\* 3.3V 로 환산 *\/ */
	/* 									+ 0.1);	/\* diode drop voltage 보상 *\/ */

	HAL_ADC_Start_DMA(&hadc1, adc_battery, SAMPLE_NBR);
}

static uint32_t sorted_data[SAMPLE_NBR];

/*********************************************************************
*	doMainBoardSensorCheck
*	메인보드 내부 센서 확인
**********************************************************************/
void doMainBoardSensorCheck(void)
{
	for (int i = 0; i < SAMPLE_NBR; i++) {
		sorted_data[i] = adc_mainBoardSensor[i * 3];
	}
	struct rtd_s *r = &ctx.rtd;
	r->adc_val = get_middle_value(sorted_data, SAMPLE_NBR);
	r->temperature = Calc_Temp_RTD(r->adc_val) + r->calibration_const;
	/* DBG_LOG("%s: rtd adc %u\r\n", __func__, r->adc_val); */

	// BOARD TEMP
	for (int i = 0; i < SAMPLE_NBR; i++) {
		sorted_data[i] = adc_mainBoardSensor[i * 3 + 1];
	}
	uint32_t temp = get_middle_value(sorted_data, SAMPLE_NBR);
	ctx.temperature = Calc_BD_Temp(temp);

	// BOARD HUMIDITY
	for (int i = 0; i < SAMPLE_NBR; i++) {
		sorted_data[i] = adc_mainBoardSensor[i * 3 + 2];
	}
	uint32_t humi = get_middle_value(sorted_data, SAMPLE_NBR);
	ctx.humidity = Calc_BD_Humi(humi);

	HAL_ADC_Start_DMA(&hadc2, adc_mainBoardSensor, SAMPLE_NBR * 3);
}

__STATIC_INLINE void swap(uint32_t *a, uint32_t *b)
{
	register uint32_t tmp = *a;
	*a = *b;
	*b = tmp;
}

static uint32_t get_middle_value(uint32_t * array, int nbr)
{
	memcpy(sorted_data, array, nbr * sizeof(uint32_t));

	for (int i = 0; i < nbr; ++i) {
		for (int j = nbr; i < j; --j) {
			if (sorted_data[j - 1] > sorted_data[j])
				swap(&sorted_data[j - 1], &sorted_data[j]);
		}
	}

	/* for (int i = 0; i < nbr / 5; i++) { */
	/* 	DBG_LOG("%u %u %u %u %u\n", sorted_data[i * 5 + 0], */
	/* 		sorted_data[i * 5 + 1], sorted_data[i * 5 + 2], */
	/* 		sorted_data[i * 5 + 3], sorted_data[i * 5 + 4]); */
	/* } */

	return sorted_data[nbr / 2];
}
