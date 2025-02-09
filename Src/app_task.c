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

/* static void send_to_external__BOARD_INFO(); */
/* void send_to_external__SLOT_INFO(struct slot_s *); */
static void send_to_external__TEMPERATURE_STATE(struct slot_s *);
static void send_to_external__TEMPERATURE(struct slot_s *);
static void check_smps(void);
static void _measuring_timer_callback(void const *arg);

static osTimerDef(periodic, _measuring_timer_callback);
static osTimerId periodic_id;

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

	periodic_id = osTimerCreate(osTimer(periodic), osTimerPeriodic, (void *)5);

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

				osDelay(500);

				FOREACH(struct slot_s *s, ctx.slots) {
					if (s->inserted) {
						existed_any_slot = true;
						last_slot_id = s->id;
					}
				}
			}

			ctx.last_slot_id = last_slot_id;
			SysProperties.InterfaceStep = STEP_READ_THRESHOLD;
			break;
		}

		case STEP_READ_THRESHOLD: {
			send_to_external(CMD_SYS, OP_STARTED_NOTIFY, NULL, 0, 12, 32);
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__THRESHOLD_REQ(s);
				osDelay(1);
			}
			SysProperties.InterfaceStep = STEP_TEMP_READ;
			break;
		}

		case STEP_TEMP_READ: { // 각 슬롯의 id 설정 완료 후 온도센서의 온도를 요청한다.
			if (!ctx.heavy_job_processing) {
				FOREACH(struct slot_s *s, ctx.slots) {
					request_to_internal__GET_TEMPERATURE_STATES(s);
					osDelay(1);
					request_to_internal__GET_TEMPERATURES(s);
					osDelay(10);
				}
			}

			/* osDelay(100); */

			/* if (SysProperties.start_flag) { */
			/* 	/\* send_to_external__BOARD_INFO(); // transmit board info *\/ */
			/* 	/\* osDelay(1); *\/ */

			/* 	if (!ctx.heavy_job_processing) { */
			/* 		FOREACH(struct slot_s *s, ctx.slots) { */
			/* 			/\* send_to_external__SLOT_INFO(s); *\/ */
			/* 			/\* osDelay(1); *\/ */
			/* 			send_to_external__TEMPERATURE_STATE(s); */
			/* 			osDelay(1); */
			/* 			send_to_external__TEMPERATURE(s); */
			/* 			osDelay(2); */
			/* 		} */
			/* 	} */
			/* } */
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

void send_to_external__BOARD_INFO()
{
	struct external_board_info info = {
		.self_id = 0, /* 부모 ID (MCU Board ID는 0으로 일단 고정) */
		.battery = ctx.battery,
		.rtd = ctx.rtd.temperature,
		.temperature = ctx.temperature,
		.humidity = ctx.humidity,
		.sd_state = ctx.sd_last_error,
	};
	send_to_external(CMD_TEMP_TEST, OP_TEMP_MAIN_INFO, &info, sizeof(info), 32, 52);
}

void send_to_external__SLOT_INFO(struct slot_s *s)
{
	if (!s) return;

	struct external_slot_info info = {
		.self_id = 0,	/* 부모 ID (MCU Board ID는 0으로 일단 고정) */
	};

	info.slot_id = s->id;
	info.slot_type = s->type;
	info.compensated = s->ntc.compensated.applied;
	send_to_external(CMD_TEMP_TEST, OP_TEMP_SLOT_INFO, &info, sizeof(info), 32, 52);
}

static void send_to_external__TEMPERATURE_STATE(struct slot_s *s)
{
	if (!s) return;

	struct external_temp_state_data d = {
		.slot_id = s->id,
	};

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

static void _measuring_timer_callback(void const *arg)
{
	/* if (!ctx.heavy_job_processing) { */
	/* 	FOREACH(struct slot_s *s, ctx.slots) { */
	/* 		send_to_external__SLOT_INFO(s); */
	/* 		request_to_internal__GET_TEMPERATURE_STATES(s); */
	/* 		/\* osDelay(1); *\/ */
	/* 		request_to_internal__GET_TEMPERATURES(s); */
	/* 		/\* osDelay(10); *\/ */
	/* 	} */
	/* } */

	if (!ctx.heavy_job_processing) {
		FOREACH(struct slot_s *s, ctx.slots) {
			send_to_external__SLOT_INFO(s);
			/* osDelay(1); */
			send_to_external__TEMPERATURE_STATE(s);
			/* osDelay(1); */
			send_to_external__TEMPERATURE(s);
			/* osDelay(2); */
		}
	}
}

void start_temperature_measuring()
{
	osTimerStart(periodic_id, SysProperties.interval_ms);
}

void stop_temperature_measuring()
{
	osTimerStop(periodic_id);
}
