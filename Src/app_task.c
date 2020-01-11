#include "app_task.h"
#include "app_ctx.h"
#include "0_GlobalValue.h"
#include "0_UartCallback.h"
#include "protocol.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"
#include "debug.h"

#include <math.h>
#include <string.h>

extern app_ctx_t ctx;

GPIO_TypeDef *SLAVE_CS_PORT[4] = {
    SLAVE_CS0_GPIO_Port, SLAVE_CS1_GPIO_Port, SLAVE_CS2_GPIO_Port,
    SLAVE_CS3_GPIO_Port}; // todo : 순서를 0123 으로 바꿔야함
uint16_t SLAVE_CS_PIN[4] = {
    SLAVE_CS0_Pin, SLAVE_CS1_Pin, SLAVE_CS2_Pin,
    SLAVE_CS3_Pin}; // back plate 의 컨넥터가 잘못 되어 있음

extern int int_tx_completed;
extern int int_rx_completed;
extern IWDG_HandleTypeDef hiwdg;

static void send_to_external__BOARD_INFO();
static void send_to_external__SLOT_INFO(struct slot_s *);
static void send_to_external__TEMPERATURE_STATE(struct slot_s *);
static void send_to_external__TEMPERATURE(struct slot_s *);
static void check_smps(void);

void app_task(void const *argument)
{
	SysProperties.InterfaceStep = STEP_SLOT_ID;
	/* SysProperties.InterfaceStep = STEP_READ_THRESHOLD; */
	/* SysProperties.InterfaceStep = STEP_TEMP_READ; */
	/* ctx.slots[0].inserted = true; */
	/* ctx.slots[2].inserted = true; */
	/* ctx.slots[3].inserted = true; */

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

				FOREACH(struct slot_s *s, ctx.slots) {
					request_to_internal__SLOT_ID_REQ(s);
					osDelay(1);
				}
				/* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
				/* 	request_to_internal__SLOT_ID_REQ(i); */
				/* 	osDelay(1); */
				/* } */

				osDelay(500);

				FOREACH(struct slot_s *s, ctx.slots) {
					if (s->inserted) {
						existed_any_slot = true;
						last_slot_id = s->id;
					}
				}
				/* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
				/* 	struct slot_s *slot = &ctx.slots[i]; */
				/* 	if (slot->inserted) { */
				/* 		existed_any_slot = true; */
				/* 		last_slot_id = i; */
				/* 	} */
				/* } */

			}
			/* if (system_reset_needed) { */
			/*     HAL_IWDG_Refresh(&hiwdg); */
			/*     osDelay(500); */
			/*     HAL_NVIC_SystemReset(); */
			/* } */

			ctx.last_slot_id = last_slot_id;
			SysProperties.InterfaceStep = STEP_READ_THRESHOLD;
			break;
		}

		case STEP_READ_THRESHOLD: //각 슬롯의 경고 온도 값을 불러 온다.
			/* startThreshold = TRUE; */
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__THRESHOLD_REQ(s);
				osDelay(1);
			}
			/* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
			/* 	struct slot_s *s = &ctx.slots[i]; */
			/* 	request_to_internal__THRESHOLD_REQ(s); */
			/* 	osDelay(1); */
			/* } */
			/* osDelay(50); */
			SysProperties.InterfaceStep = STEP_TEMP_READ;
			break;

		case STEP_TEMP_READ: { // 각 슬롯의 id 설정 완료 후 온도센서의 온도를 요청한다.
			if (!ctx.hard_job_processing) {
				FOREACH(struct slot_s *s, ctx.slots) {
					request_to_internal__TEMPERATURE_STATE_REQ(s);
					osDelay(1);
					request_to_internal__TEMPERATURE_REQ(s);
					osDelay(10);
				}
				/* struct slot_s *s; */
				/* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
				/* 	s = &ctx.slots[i]; */
				/* 	request_to_internal__TEMPERATURE_STATE_REQ(s); */
				/* 	osDelay(1); */
				/* 	request_to_internal__TEMPERATURE_REQ(s); */
				/* 	osDelay(10); */
				/* } */
			}
			/* next_wait += SysProperties.interval_ms * 1000; */
			osDelay(100);

			if (SysProperties.start_flag) {
				send_to_external__BOARD_INFO(); // transmit board info
				osDelay(1);

				if (!ctx.hard_job_processing) {
					FOREACH(struct slot_s *s, ctx.slots) {
						send_to_external__SLOT_INFO(s);
						osDelay(1);
						send_to_external__TEMPERATURE_STATE(s);
						osDelay(1);
						send_to_external__TEMPERATURE(s);
						osDelay(2);
					}
					/* struct slot_s *s; */
					/* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
					/* 	s = &ctx.slots[i]; */
					/* 	send_to_external__SLOT_INFO(s->id); */
					/* 	osDelay(1); */
					/* 	send_to_external__TEMPERATURE_STATE(s->id); */
					/* 	osDelay(1); */
					/* 	send_to_external__TEMPERATURE(s->id); */
					/* 	osDelay(2); */
					/* } */
				}
			}
			osDelay(25);
			check_smps();
			osDelayUntil(&next_wait, SysProperties.interval_ms);
			break;
		}
		}
	}
}

static void check_smps(void)
{
	SysProperties.smpsState = 0;
	if (HAL_GPIO_ReadPin(SMPS1_DECTECT_GPIO_Port, SMPS1_DECTECT_Pin) == GPIO_PIN_RESET)
		SysProperties.smpsState |= 0x01;

	if (HAL_GPIO_ReadPin(SMPS2_DECTECT_GPIO_Port, SMPS2_DECTECT_Pin) == GPIO_PIN_RESET)
		SysProperties.smpsState |= 0x02;
}

static void send_to_external__BOARD_INFO()
{
	struct external_board_info info = {
		.self_id = 0, /* 부모 ID (MCU Board ID는 0으로 일단 고정) */
		.sd_state = sdValue.sdState,
	};
	memcpy(info.values, &TestData.mainBoard[MBS_BATTERY].UI8[0], 16);
	send_to_external(CMD_TEMP_TEST, OP_TEMP_MAIN_INFO, &info, sizeof(info), 32, 52);
}

static void send_to_external__SLOT_INFO(struct slot_s *s)
{
	if (!s) return;

	struct external_slot_info info = {
		.self_id = 0,	/* 부모 ID (MCU Board ID는 0으로 일단 고정) */
	};

	info.slot_id = s->id;
	info.slot_type = s->type;
	info.revision_applied = TestData.revisionApply[s->id];
	send_to_external(CMD_TEMP_TEST, OP_TEMP_SLOT_INFO, &info, sizeof(info), 32, 52);
}

static void send_to_external__TEMPERATURE_STATE(struct slot_s *s)
{
	if (!s) return;

	struct external_temp_state_data d = {
		.slot_id = s->id,
	};

	/* struct slot_s *s = &ctx.slots[slot_id]; */
	memcpy(d.states, s->ntc.channel_states, sizeof(d.states));
	send_to_external(CMD_TEMP_TEST, OP_TEMP_CHANNEL_INFO, &d, 33, 36, 56);
}

static struct external_temp_data temp_data = { 0 };

static void send_to_external__TEMPERATURE(struct slot_s *s)
{
	if (!s) return;

	temp_data.slot_id = s->id;
	memcpy(temp_data.values, s->ntc.temperatures, sizeof(temp_data.values));

	send_to_external(CMD_TEMP_TEST, OP_TEMP_CHANNEL_VALUE,
			&temp_data, sizeof(temp_data), 132, 152);
}
