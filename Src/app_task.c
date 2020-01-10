#include "app_task.h"
#include "0_GlobalValue.h"
#include "0_UartCallback.h"
#include "protocol.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"
#include "debug.h"

#include <math.h>
#include <string.h>


GPIO_TypeDef *SLAVE_CS_PORT[4] = {
    SLAVE_CS0_GPIO_Port, SLAVE_CS1_GPIO_Port, SLAVE_CS2_GPIO_Port,
    SLAVE_CS3_GPIO_Port}; // todo : 순서를 0123 으로 바꿔야함
uint16_t SLAVE_CS_PIN[4] = {
    SLAVE_CS0_Pin, SLAVE_CS1_Pin, SLAVE_CS2_Pin,
    SLAVE_CS3_Pin}; // back plate 의 컨넥터가 잘못 되어 있음

extern int int_tx_completed;
extern int int_rx_completed;
extern IWDG_HandleTypeDef hiwdg;

void app_task(void const *argument)
{
	SysProperties.InterfaceStep = STEP_SLOT_ID;
	/* SysProperties.InterfaceStep = STEP_READ_THRESHOLD; */
	/* SysProperties.InterfaceStep = STEP_TEMP_READ; */
	/* SysProperties.slots[0].inserted = true; */
	/* SysProperties.slots[2].inserted = true; */
	/* SysProperties.slots[3].inserted = true; */

	HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLAVE_OE1_GPIO_Port, SLAVE_OE1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_EN_BT_GPIO_Port, UART_EN_BT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_SET);

	DoSlotReset(ALL_SLOT);
	DoRejectSlot();
	HAL_IWDG_Refresh(&hiwdg);
	osDelay(100);
	HAL_IWDG_Refresh(&hiwdg);
	// todo : 센서 타입을 슬레이브 보드에서 읽어 와야 함.

	uint32_t next_wait = osKernelSysTick();

	for (;;) {
		switch (SysProperties.InterfaceStep) {

		case STEP_SLOT_ID: { // 부팅 하면 각 슬롯의 id 를 지정 한다. id 는
			volatile bool existed_any_slot = false;
			__IO uint8_t last_slot_id = 0;

			while (!existed_any_slot) {
				/* check_slots_inserted(SysProperties.slots, MAX_SLOT_NUM); */

				for (int i = 0; i < MAX_SLOT_NUM; i++) {
					send_slot_id_req(i);
					osDelay(1);
				}

				osDelay(500);

				for (int i = 0; i < MAX_SLOT_NUM; i++) {
					struct slot_properties_s *slot = &SysProperties.slots[i];
					if (slot->inserted) {
						existed_any_slot = true;
						last_slot_id = i;
					}
				}

			}
			/* if (system_reset_needed) { */
			/*     HAL_IWDG_Refresh(&hiwdg); */
			/*     osDelay(500); */
			/*     HAL_NVIC_SystemReset(); */
			/* } */

			SysProperties.last_slot_id = last_slot_id;
			SysProperties.InterfaceStep = STEP_READ_THRESHOLD;
			break;
		}

		case STEP_READ_THRESHOLD: //각 슬롯의 경고 온도 값을 불러 온다.
			/* startThreshold = TRUE; */
			for (int i = 0; i < MAX_SLOT_NUM; i++) {
				struct slot_properties_s *slot = &SysProperties.slots[i];
				DoThresholdReq(slot);
				/* osDelay(10); */
			}
			osDelay(50);
			SysProperties.InterfaceStep = STEP_TEMP_READ;
			break;

		case STEP_TEMP_READ: { // 각 슬롯의 id 설정 완료 후 온도센서의 온도를 요청한다.
			for (int i = 0; i < MAX_SLOT_NUM; i++) {
				struct slot_properties_s *slot = &SysProperties.slots[i];
				/* DBG_LOG("slot->id: %d, type: %d, inserted: %d\n", */
				/*    slot->id, slot->type, slot->inserted); */
				DoReqTemperatureState(slot);
				/* osDelay(20); */
				DoReqTemperature(slot);
				osDelay(20);
			}
			/* next_wait += SysProperties.interval_ms * 1000; */
			osDelay(200);

			if (SysProperties.start_flag) {
				DoMCUboardInfo(); // transmit board info
				/* osDelay(10); */

				struct slot_properties_s *s;
				for (int i = 0; i < MAX_SLOT_NUM; i++) {
					s = &SysProperties.slots[i];
					DoSlotInfo(s->id);
					/* osDelay(10); */
					DoChannelInfo(s->id);
					/* osDelay(10); */
					DoChannelValue(s->id);
					osDelay(30);
				}
			}
			DoSmpsCheck();
			osDelayUntil(&next_wait, SysProperties.interval_ms);
			break;
		}
		}
	}
}
