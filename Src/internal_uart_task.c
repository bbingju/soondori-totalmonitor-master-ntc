#include "internal_uart_task.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "frame.h"
#include "app_ctx.h"
#include "internal_job_task.h"
#include "job_task.h"
#include "fs_task.h"
#include "app_task.h"
#include "external_uart_task.h"
#include "0_Util.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "debug.h"

#include <string.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

extern DMA_HandleTypeDef hdma_usart2_rx;
extern int transaction_completed;
extern app_ctx_t ctx;

static osMessageQDef(internal_rx_q, 48, uint32_t);
static osMessageQId(internal_rx_q_id);

void internal_rx_notify() { osMessagePut(internal_rx_q_id, 1, 0); }

volatile int int_tx_completed = 1;

static uint8_t internal_rx_buffer[384 + 128];

static void internal_rx_check(void);
static void parse_rx(const void *data, size_t len);

static void handle__THRESHOLDS(struct internal_frame *);
static void handle__SET_VARIATION(struct internal_frame *);
static void handle__GET_VARIATION(struct internal_frame *);
static void handle__TEMPERATURES(struct internal_frame *);
static void handle__TEMPERATURE_STATES(struct internal_frame *);
static void handle__COMPENSATED(struct internal_frame *);
static void DoAnsRevisionConstantSet(struct internal_frame *); /* NOT_USED */
static void DoAnsRevisionConstantReq(struct internal_frame *); /* NOT_USED */
static void handle__SET_COMPENSATED_TR_CONST(struct internal_frame *);
static void handle__GET_COMPENSATED_TR_CONST(struct internal_frame *);
static void handle__CAL_NTC_TBL(struct internal_frame *);
static void handle__GET_NTC_TBL(struct internal_frame *);
static void DoAnsCalibrationNTCConstantSet(struct internal_frame *); /* NOT_USED */
static void DoAnsCalibrationNTCConstantReq(struct internal_frame *); /* NOT_USED */

void response_from_internal(struct internal_frame *received)
{
	/* if (received->cmd == INTERNAL_CMD_THRESHOLD_SET || received->cmd ==
	 * INTERNAL_CMD_THRESHOLD_REQ) { */
	DBG_LOG("int RX slot %d: %s - (datalen: %d) \n",
		received->slot_id, int_cmd_str(received->cmd), received->datalen);
	/* DBG_DUMP(received->data, received->datalen); */
	/* } */

	switch (received->cmd) {
	case INTERNAL_CMD_BOARD_TYPE_REQ:
		ctx.slots[received->slot_id].type = received->data[0];
		break;
		/* case CMD_BOARD_EN_REQ: */
		/*     break; */
		/* case CMD_BOARD_EN_SET: */
		/*     break; */
	case INTERNAL_CMD_SLOT_ID_REQ:
		ctx.slots[received->slot_id].inserted = TRUE;
		ctx.slots[received->slot_id].type = SLOT_TYPE_NTC;
		break;
		/* case CMD_HW_VER: */
		/*     break; */
		/* case CMD_FW_VER: */
		/*     break; */
		/* case CMD_UUID_REQ: */
		/*     break; */
	case INTERNAL_CMD_ADC_REQ:
		break;
	case INTERNAL_CMD_RELAY_REQ:
		break;
	case INTERNAL_CMD_RELAY_SET:
		break;
	case INTERNAL_CMD_COMPENSATED_SET:
	case INTERNAL_CMD_COMPENSATED_REQ:
		handle__COMPENSATED(received);
		break;
	case INTERNAL_CMD_COMPENSATE_CONSTANT_SET:
		DoAnsRevisionConstantSet(received);
		break;
	case INTERNAL_CMD_COMPENSATE_CONSTANT_REQ:
		DoAnsRevisionConstantReq(received);
		break;
	case INTERNAL_CMD_COMPENSATE_TR_CONST_SET:
		handle__SET_COMPENSATED_TR_CONST(received);
		break;
	case INTERNAL_CMD_COMPENSATE_TR_CONST_REQ:
		handle__GET_COMPENSATED_TR_CONST(received);
		break;
	case INTERNAL_CMD_CORRECTION_NTC_CONSTANT_SET:
		DoAnsCalibrationNTCConstantSet(received);
		break;
	case INTERNAL_CMD_CORRECTION_NTC_CONSTANT_REQ:
		DoAnsCalibrationNTCConstantReq(received);
		break;
	case INTERNAL_CMD_TEMPERATURE_STATE_REQ:
		handle__TEMPERATURE_STATES(received);
		break;
	case INTERNAL_CMD_TEMPERATURE_REQ:
		handle__TEMPERATURES(received);
		break;
	case INTERNAL_CMD_THRESHOLD_REQ:
	case INTERNAL_CMD_THRESHOLD_SET:
		handle__THRESHOLDS(received);
		break;
	case INTERNAL_CMD_VARIATION_REQ:
		handle__GET_VARIATION(received);
		break;
	case INTERNAL_CMD_VARIATION_SET:
		handle__SET_VARIATION(received);
		break;
	case INTERNAL_CMD_CORRECTION_NTC_TABLE_CAL:
		handle__CAL_NTC_TBL(received);
		break;
	case INTERNAL_CMD_CORRECTION_NTC_TABLE_REQ:
		handle__GET_NTC_TBL(received);
		break;
	}
}

void internal_rx_task(void const *arg)
{
	internal_rx_q_id = osMessageCreate(osMessageQ(internal_rx_q), NULL);
	HAL_UART_Receive_DMA(&huart2, internal_rx_buffer, ARRAY_LEN(internal_rx_buffer));

	while (1) {
		/* HAL_UART_Receive_DMA(&huart2, internal_rx_buffer, ARRAY_LEN(internal_rx_buffer)); */
		osEvent e = osMessageGet(internal_rx_q_id, osWaitForever);
		if (e.status == osEventMessage)
			internal_rx_check();
	}
}

/**
 * @brief  Check for new data received with DMA
 */
static void internal_rx_check(void)
{
	static size_t old_pos;
	size_t pos;

	/* Calculate current position in buffer */
	pos = ARRAY_LEN(internal_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	/* pos = ARRAY_LEN(internal_rx_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5); */
	if (pos != old_pos) {    /* Check change in received data */
		if (pos > old_pos) { /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			parse_rx(&internal_rx_buffer[old_pos], pos - old_pos);
		} else {
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			parse_rx(&internal_rx_buffer[old_pos], ARRAY_LEN(internal_rx_buffer) - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				parse_rx(&internal_rx_buffer[0], pos);
			}
		}
	}
	old_pos = pos; /* Save current position as old */

	/* Check and manually update if we reached end of buffer */
	if (old_pos == ARRAY_LEN(internal_rx_buffer)) {
		old_pos = 0;
	}
}

/**
 * @brief           Process received data over UART
 * @note            Either process them directly or copy to other bigger buffer
 * @param[in]       data: Data to process
 * @param[in]       len: Length in units of bytes
 */
static void parse_rx(const void *data, size_t len)
{
	static  struct internal_frame frm = {0};

	for (int i = 0; i < len; i++) {
		if (parse_internal_frame(&frm, data + i)) {
			transaction_completed = 1;
			post_job(JOB_TYPE_FROM_INTERNAL, &frm, sizeof(frm));
		}
	}
}

void request_to_internal__SLOT_ID_REQ(struct slot_s *s)
{
	HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
	DoRejectSlot();

    	struct internal_frame data = {
		.slot_id = s->id,
		.cmd = INTERNAL_CMD_SLOT_ID_REQ,
		.datalen = 0,
		.data = { 0 },
        };
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

static void handle__TEMPERATURES(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		memcpy(s->ntc.old_temperatures, s->ntc.temperatures, sizeof(float) * CHANNEL_NBR);
		memcpy(s->ntc.temperatures, msg->temperatures.v, sizeof(float) * CHANNEL_NBR);

		/* write log */
		if (ctx.last_slot_id == msg->slot_id)
			post_fs_job(FS_JOB_TYPE_SAVE_LOG);
	}
}

static void handle__TEMPERATURE_STATES(struct internal_frame *frm)
{
	if (frm && frm->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[frm->slot_id];
		memcpy(&s->ntc.channel_states[0], &frm->channel_states, sizeof(frm->channel_states));
		memcpy(&s->ntc.channel_states[16], &frm->channel_states, sizeof(frm->channel_states));
	}
}

static void handle__THRESHOLDS(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];

		memcpy(s->ntc.thresholds, &msg->thresholds, sizeof(float) * 32);

		struct external_thresholds data = {
			.slot_id = msg->slot_id,
		};
		memcpy(data.values, s->ntc.thresholds, sizeof(float) * 32);

		uint8_t option = (msg->cmd == INTERNAL_CMD_THRESHOLD_SET) ?
			OP_WARNING_THRESHOLD_SET : OP_WARNING_THRESHOLD_REQ;
		send_to_external(CMD_WARNING, option, &data, 129, 132, 152);
	}
}

static void handle__SET_VARIATION(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];

		struct external_variation data = {
			.slot_id = s->id,
			.value = msg->variation_set.value,
		};

		send_to_external(CMD_WARNING, OP_WARNING_VARIATION_SET, &data, 5, 12, 32);
	}
}

static void handle__GET_VARIATION(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];

		struct external_variation data = {
			.slot_id = s->id,
			.value = msg->variation_get.value,
		};

		send_to_external(CMD_WARNING, OP_WARNING_VARIATION_REQ, &data, 5, 12, 32);
	}
}

static void handle__COMPENSATED(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.compensated.applied = msg->compensated.v;

		struct external_compensated data = {
			.slot_id = s->id,
			.v = s->ntc.compensated.applied,
		};

		uint8_t option = (msg->cmd == INTERNAL_CMD_COMPENSATED_SET) ?
			OP_COMPENSATED_APPLY_SET : OP_COMPENSATED_APPLY_REQ;
		send_to_external(CMD_COMPENSATE, option, &data, 2, 12, 32);
	}
}

static void DoAnsRevisionConstantSet(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.compensated.contact_const = msg->revision_const.v;

		struct external_revision_const data = {
			.slot_id = s->id,
			.v = s->ntc.compensated.contact_const,
		};

		send_to_external(CMD_COMPENSATE, OP_COMPENSATED_CONSTANT_SET, &data, 6, 12, 32);
	}
}

static void DoAnsRevisionConstantReq(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.compensated.contact_const = msg->revision_const.v;

		struct external_revision_const data = {
			.slot_id = s->id,
			.v = s->ntc.compensated.contact_const,
		};

		send_to_external(CMD_COMPENSATE, OP_COMPENSATED_CONSTANT_REQ, &data, 6, 12, 32);
	}
}

static void handle__SET_COMPENSATED_TR_CONST(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.compensated.tr1 = msg->revision_tr_const.r1;
		s->ntc.compensated.tr2 = msg->revision_tr_const.r2;

		struct external_revision_tr_const data = {
			.slot_id = s->id,
			.r1 = s->ntc.compensated.tr1,
			.r2 = s->ntc.compensated.tr2,
		};

		send_to_external(CMD_COMPENSATE, OP_COMPENSATED_TR_CONST_SET, &data, 9, 12, 32);
	}
}

static void handle__GET_COMPENSATED_TR_CONST(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.compensated.tr1 = msg->revision_tr_const.r1;
		s->ntc.compensated.tr2 = msg->revision_tr_const.r2;

		struct external_revision_tr_const data = {
			.slot_id = s->id,
			.r1 = s->ntc.compensated.tr1,
			.r2 = s->ntc.compensated.tr2,
		};

		send_to_external(CMD_COMPENSATE, OP_COMPENSATED_TR_CONST_REQ, &data, 9, 12, 32);
	}
}

static void handle__CAL_NTC_TBL(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		memcpy(s->ntc.calibration_tbl, msg->data, sizeof(float) * 32);

		uint8_t calData[130] = { 0 };
		calData[0] = s->id;
		memcpy(&calData[1], s->ntc.calibration_tbl, 128);
		send_to_external(CMD_CORRECTION, OP_CORRECTION_NTC_CON_TABLE_CAL, calData, 129, 132, 152);
	}
}

static void handle__GET_NTC_TBL(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		memcpy(s->ntc.calibration_tbl, msg->data, sizeof(float) * 32);

		uint8_t calData[130] = { 0 };
		calData[0] = s->id;
		memcpy(&calData[1], s->ntc.calibration_tbl, 128);
		send_to_external(CMD_CORRECTION, OP_CORRECTION_NTC_CON_TABLE_REQ, calData, 129, 132, 152);
	}
}

static void DoAnsCalibrationNTCConstantSet(struct internal_frame *msg)
{
#if 0
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.calibration_const = *((float *)&msg->data[0]);

		send_to_external(CMD_CORRECTION, OP_CORRECTION_NTC_CONSTANT_SET,
				&s->ntc.calibration_const, 4, 12, 32);
	}
#endif
}

static void DoAnsCalibrationNTCConstantReq(struct internal_frame *msg)
{
#if 0
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		s->ntc.calibration_const = *((float *)&msg->data[0]);

		send_to_external(CMD_CORRECTION, OP_CORRECTION_NTC_CONSTANT_REQ,
				&s->ntc.calibration_const, 4, 12, 32);
	}
#endif
}


/*********************************************************************
 *       doSlotReset
 *       SLOT 통신 문제가 발생 됬을 때 SLOT 강제 리셋 시킴
 *   slot : SLOT 번호, ALL_SLOT : 모든 SLOT RESET
 **********************************************************************/
void DoSlotReset(uint8_t slot)
{
    /*    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
       //buffer ic on

        if(slot == ALL_SLOT)
        {
            HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(SLAVE_CS_PORT[slot], SLAVE_CS_PIN[slot],
       GPIO_PIN_SET);
        }

        HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin,
       GPIO_PIN_SET); HAL_Delay(70); HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port,
       SLAVE_OE_Pin, GPIO_PIN_RESET);
        */
}

/*********************************************************************
 *       doSelectSlot
 *       SLOT 선택 하는 함수
 *   slot : SLOT 번호
 **********************************************************************/
void DoSelectSlot(uint8_t slot)
{
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin,
                      slot == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin,
                      slot == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin,
                      slot == 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin,
                      slot == 3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*********************************************************************
 *       doRejectSlot
 *       SLOT 선택 제거 함수
 **********************************************************************/
void DoRejectSlot(void)
{
    HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
**********************************************************************
*
*       Tx 관련 함수
*
*   doSet 으로 시작 하는 함수 : slot에 설정을 하는 함수
*   doReq 으로 시작 하는 함수 : slot에 값을 요청 하는 함수
**********************************************************************
**********************************************************************/

void request_to_internal__CALIBRATION_NTC_TABLE_CAL(struct slot_s *s)
{
	if (s && s->inserted) {
		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_CORRECTION_NTC_TABLE_CAL,
			.datalen = sizeof(float),
		};
		*((float *)&frm.data[0]) = ctx.rtd.temperature;
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
}

void request_to_internal__CALIBRATION_NTC_TABLE_REQ(struct slot_s *s)
{
	if (s && s->inserted) {
		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_CORRECTION_NTC_TABLE_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
}

void request_to_internal__CALIBRATION_NTC_CONST_SET(struct slot_s *s)
{
#if 0
	if (s && s->inserted) {
		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_CORRECTION_NTC_CONSTANT_SET,
			.datalen = 4,
		};
		*((float *)&frm.data[0]) = s->ntc.calibration_const;
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
#endif
}


void request_to_internal__CALIBRATION_NTC_CONST_REQ(struct slot_s *s)
{
#if 0
	if (s && s->inserted) {
		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_CORRECTION_NTC_CONSTANT_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
#endif
}

void request_to_internal__THRESHOLD_SET(struct slot_s *slot,
					uint8_t channel, float threshold)
{
	if (slot && slot->inserted) {
		struct internal_frame frm = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_THRESHOLD_SET,
			.datalen = 5,
		};
		frm.data[0] = channel;
		*((float *) &frm.data[1]) = threshold;

		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, sizeof(frm));
	}
}

void request_to_internal__THRESHOLD_REQ(struct slot_s *slot)
{
	if (slot && slot->inserted) {
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_THRESHOLD_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

void request_to_internal__VARIATION_SET(struct slot_s *slot, float variation)
{
	if (slot && slot->inserted) {
		struct internal_frame frm = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_VARIATION_SET,
			.datalen = 4,
		};
		*((float *) &frm.data[0]) = variation;

		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, sizeof(frm));
	}
}

void request_to_internal__VARIATION_REQ(struct slot_s *slot)
{
	if (slot && slot->inserted) {
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_VARIATION_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

/**
 * request_to_internal__GET_TEMPERATURES
 */
void request_to_internal__GET_TEMPERATURES(struct slot_s *slot)
{
	if (slot && slot->inserted) {
		/* HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET); */
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_TEMPERATURE_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

void request_to_internal__GET_TEMPERATURE_STATES(struct slot_s *slot)
{
	if (slot && slot->inserted) {
		/* HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET); */
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_TEMPERATURE_STATE_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

 void request_to_internal__SET_COMPENSATED(struct slot_s *s, uint8_t const mode)
{
	if (!s) return;

	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame frm = {
		.slot_id = s->id,
		.cmd = INTERNAL_CMD_COMPENSATED_SET,
		.datalen = 1,
		.compensated.v = mode,
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
}

void request_to_internal__GET_COMPENSATED(struct slot_s *s)
{
	if (s && s->inserted) {
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_COMPENSATED_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
}

void request_to_internal__SET_COMPENSATED_CONTACT_CONST(struct slot_s *s)
{
	if (s && s->inserted) {
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_COMPENSATE_CONSTANT_SET,
			.datalen = 4,
		};
		*((float *) &frm.data[0]) = s->ntc.compensated.contact_const;
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, frm.datalen + 3);
	}
}


void request_to_internal__GET_COMPENSATED_CONTACT_CONST(struct slot_s *s)
{
	if (s && s->inserted) {
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_COMPENSATE_CONSTANT_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, frm.datalen + 3);
	}
}

void request_to_internal__SET_COMPENSATED_TR_CONST(struct slot_s *s, float r1, float r2)
{
	if (s && s->inserted) {
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_COMPENSATE_TR_CONST_SET,
			.datalen = 8,
		};
		frm.revision_tr_const.r1 = r1;
		frm.revision_tr_const.r2 = r2;

		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
	}
}

void request_to_internal__GET_COMPENSATED_TR_CONST(struct slot_s *s)
{
	if (s && s->inserted) {
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

		struct internal_frame frm = {
			.slot_id = s->id,
			.cmd = INTERNAL_CMD_COMPENSATE_TR_CONST_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, frm.datalen + 3);
	}
}
