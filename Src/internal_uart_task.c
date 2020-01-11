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

extern int transaction_completed;
extern app_ctx_t ctx;

static osMessageQDef(internal_rx_q, 48, uint32_t);
static osMessageQId(internal_rx_q_id);

void internal_rx_notify() { osMessagePut(internal_rx_q_id, 1, 0); }

int int_tx_completed = 1;

static uint8_t internal_rx_buffer[384 + 128];

static void internal_rx_check(void);
static void parse_rx(const void *data, size_t len);

static void DoAnsBoardType(struct internal_frame *);
static void handle_threshold(struct internal_frame *);
static void handle_temperature(struct internal_frame *);
static void handle_temerature_state(struct internal_frame *);
static void DoAnsRevisionApplySet(struct internal_frame *);
static void DoAnsRevisionApplyReq(struct internal_frame *);
static void DoAnsRevisionConstantSet(struct internal_frame *);
static void DoAnsRevisionConstantReq(struct internal_frame *);
static void DoAnsCalibrationNTCTableCal(struct internal_frame *);
static void DoAnsCalibrationNTCTableReq(struct internal_frame *);
static void DoAnsCalibrationNTCConstantSet(struct internal_frame *);
static void DoAnsCalibrationNTCConstantReq(struct internal_frame *);

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
		DoAnsBoardType(received);
		break;
		/* case CMD_BOARD_EN_REQ: */
		/*     break; */
		/* case CMD_BOARD_EN_SET: */
		/*     break; */
	case INTERNAL_CMD_SLOT_ID_REQ:
		/* noReturnSendCt = 0; */
		/* if (SendSlotNumber == received->slot_id) { */
			ctx.slots[received->slot_id].inserted = TRUE;
			/* DoIncSlotIdStep(SendSlotNumber); */
		/* } */
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
	case INTERNAL_CMD_REVISION_APPLY_SET:
		DoAnsRevisionApplySet(received);
		break;
	case INTERNAL_CMD_REVISION_CONSTANT_SET:
		DoAnsRevisionConstantSet(received);
		break;
	case INTERNAL_CMD_REVISION_APPLY_REQ:
		DoAnsRevisionApplyReq(received);
		break;
	case INTERNAL_CMD_REVISION_CONSTANT_REQ:
		DoAnsRevisionConstantReq(received);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_SET:
		DoAnsCalibrationNTCConstantSet(received);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_REQ:
		DoAnsCalibrationNTCConstantReq(received);
		break;
	case INTERNAL_CMD_TEMPERATURE_STATE_REQ:
		handle_temerature_state(received);
		break;
	case INTERNAL_CMD_TEMPERATURE_REQ:
		handle_temperature(received);
		break;
	case INTERNAL_CMD_THRESHOLD_REQ:
	case INTERNAL_CMD_THRESHOLD_SET:
		handle_threshold(received);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_CAL:
		DoAnsCalibrationNTCTableCal(received);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_REQ:
		DoAnsCalibrationNTCTableReq(received);
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
	pos = ARRAY_LEN(internal_rx_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
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

static void DoAnsBoardType(struct internal_frame *msg)
{
	if (msg)
		ctx.slots[msg->slot_id].type = msg->data[0];
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

static void handle_temperature(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];
		memcpy(s->ntc.temperatures, msg->temperatures.v, sizeof(float) * 32);

		/* write log */
		if (ctx.last_slot_id == msg->slot_id)
			post_fs_job(FS_JOB_TYPE_SAVE_LOG);
	}
}

static void handle_temerature_state(struct internal_frame *frm)
{
	if (frm && frm->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[frm->slot_id];
		memcpy(&s->ntc.channel_states[0], &frm->channel_states, sizeof(frm->channel_states));
		memcpy(&s->ntc.channel_states[16], &frm->channel_states, sizeof(frm->channel_states));
	}
}

static void handle_threshold(struct internal_frame *msg)
{
	if (msg && msg->slot_id < MAX_SLOT_NUM) {
		struct slot_s *s = &ctx.slots[msg->slot_id];

		memcpy(s->ntc.thresholds, &msg->thresholds, sizeof(float) * 32);

		uint8_t thresholdData[130] = {0};
		thresholdData[0] = msg->slot_id;
		memcpy(&thresholdData[1], s->ntc.thresholds, sizeof(float) * 32);
		send_to_external(CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ,
				thresholdData, 129, 132, 152);
	}
}

static void DoAnsRevisionApplySet(struct internal_frame *msg)
{
	if (!msg)
		return;

	TestData.revisionApply[msg->slot_id] = msg->data[0];

	static uint8_t revision_apply[2] = {0};
	revision_apply[0] = msg->slot_id;
	revision_apply[1] = TestData.revisionApply[msg->slot_id];
	send_to_external(CMD_REVISION, OP_REVISION_APPLY_SET, revision_apply, 2, 12, 32);
}

static void DoAnsRevisionApplyReq(struct internal_frame *msg)
{
	if (!msg)
		return;

        TestData.revisionApply[msg->slot_id] = msg->data[0];

	uint8_t data[2];
	data[0] = msg->slot_id;
	data[1] = TestData.revisionApply[msg->slot_id];

	send_to_external(CMD_REVISION, OP_REVISION_APPLY_REQ, data, 2, 12, 32);
}

static void DoAnsRevisionConstantSet(struct internal_frame *msg)
{
	if (!msg)
		return;

        TestData.revisionConstant[msg->slot_id].UI8[0] = msg->data[0];
        TestData.revisionConstant[msg->slot_id].UI8[1] = msg->data[1];
        TestData.revisionConstant[msg->slot_id].UI8[2] = msg->data[2];
        TestData.revisionConstant[msg->slot_id].UI8[3] = msg->data[3];

	uint8_t data[5] = {0};
	data[0] = msg->slot_id;
	data[1] = TestData.revisionConstant[msg->slot_id].UI8[0];
	data[2] = TestData.revisionConstant[msg->slot_id].UI8[1];
	data[3] = TestData.revisionConstant[msg->slot_id].UI8[2];
	data[4] = TestData.revisionConstant[msg->slot_id].UI8[3];
	send_to_external(CMD_REVISION, OP_REVISION_CONSTANT_SET, data, 6, 12, 32);
}

static void DoAnsRevisionConstantReq(struct internal_frame *msg)
{
	if (!msg)
		return;

        TestData.revisionConstant[msg->slot_id].UI8[0] = msg->data[0];
        TestData.revisionConstant[msg->slot_id].UI8[1] = msg->data[1];
        TestData.revisionConstant[msg->slot_id].UI8[2] = msg->data[2];
        TestData.revisionConstant[msg->slot_id].UI8[3] = msg->data[3];

	uint8_t data[5];
	data[0] = msg->slot_id;
	data[1] = TestData.revisionConstant[msg->slot_id].UI8[0];
	data[2] = TestData.revisionConstant[msg->slot_id].UI8[1];
	data[3] = TestData.revisionConstant[msg->slot_id].UI8[2];
	data[4] = TestData.revisionConstant[msg->slot_id].UI8[3];
	send_to_external(CMD_REVISION, OP_REVISION_CONSTANT_REQ, data, 6, 12, 32);
}

static void DoAnsCalibrationNTCTableCal(struct internal_frame *msg)
{
	if (!msg)
		return;

        for (int i = 0; i < 32; i++) {
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[0] = msg->data[i * 4 + 0];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[1] = msg->data[i * 4 + 1];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[2] = msg->data[i * 4 + 2];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[3] = msg->data[i * 4 + 3];
        }

	uint8_t calData[130] = { 0 };
	calData[0] = msg->slot_id;
	memcpy((void *)&calData[1],
		(void *)&TestData.ntcCalibrationTable[msg->slot_id][0].UI8[0], 128);
	send_to_external(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL,
			calData, 129, 132, 152);
}

static void DoAnsCalibrationNTCConstantSet(struct internal_frame *msg)
{
	if (!msg)
		return;

	TestData.ntcCalibrationConst.UI8[0] = msg->data[0];
	TestData.ntcCalibrationConst.UI8[1] = msg->data[1];
	TestData.ntcCalibrationConst.UI8[2] = msg->data[2];
	TestData.ntcCalibrationConst.UI8[3] = msg->data[3];

	send_to_external(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET,
			&TestData.ntcCalibrationConst.UI8[0], 4, 12, 32);
}

static void DoAnsCalibrationNTCTableReq(struct internal_frame *msg)
{
	if (!msg)
		return;

        for (int i = 0; i < 32; i++) {
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[0] = msg->data[i * 4 + 0];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[1] = msg->data[i * 4 + 1];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[2] = msg->data[i * 4 + 2];
		TestData.ntcCalibrationTable[msg->slot_id][i].UI8[3] = msg->data[i * 4 + 3];
        }

	uint8_t data[130] = {0};
	data[0] = msg->slot_id;
	memcpy(&data[1], &TestData.ntcCalibrationTable[msg->slot_id][0].UI8[0], 128);
	send_to_external(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ,
			data, 129, 132, 152);
}

static void DoAnsCalibrationNTCConstantReq(struct internal_frame *msg)
{
	if (!msg)
		return;

        TestData.ntcCalibrationConst.UI8[0] = msg->data[0];
        TestData.ntcCalibrationConst.UI8[1] = msg->data[1];
        TestData.ntcCalibrationConst.UI8[2] = msg->data[2];
        TestData.ntcCalibrationConst.UI8[3] = msg->data[3];

	send_to_external(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ,
			&TestData.ntcCalibrationConst.UI8[0], 4, 12, 32);
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

void DoCalibrationNTCTableCal(uint8_t slotNumber)
{
	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_CAL,
		.datalen = 4,
		/* .data = &TestData.mainBoard[MBS_RTD].UI8[0], */
        };
	memcpy(data.data, &TestData.mainBoard[MBS_RTD].UI8[0], 4);
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

void DoCalibrationNTCConstantSet(uint8_t slotNumber)
{
	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_SET,
		.datalen = 4,
		/* .data = &TestData.ntcCalibrationConst.UI8[0], */
        };
	memcpy(data.data, &TestData.ntcCalibrationConst.UI8[0], 4);
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

void DoCalibrationNTCTableReq(uint8_t slotNumber)
{
	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_REQ,
		.datalen = 0,
		.data = { 0 },
        };
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

void DoCalibrationNTCConstantReq(uint8_t slotNumber)
{
	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_REQ,
		.datalen = 0,
		.data = { 0 },
        };
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
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

/**
 * request_to_internal__TEMPERATURE_REQ
 */
void request_to_internal__TEMPERATURE_REQ(struct slot_s *slot)
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

void request_to_internal__TEMPERATURE_STATE_REQ(struct slot_s *slot)
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

 void request_to_internal__REVISION_APPLY_SET(struct slot_s *s, uint8_t const mode)
{
	if (!s) return;

	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame frm = {
		.slot_id = s->id,
		.cmd = INTERNAL_CMD_REVISION_APPLY_SET,
		.datalen = 1,
		.revision_apply.enabled = mode,
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
}

void request_to_internal__REVISION_APPLY_REQ(struct slot_s *s)
{
	if (!s) return;

	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame frm = {
		.slot_id = s->id,
		.cmd = INTERNAL_CMD_REVISION_APPLY_REQ,
		.datalen = 0,
		.data = { 0 },
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &frm, 3 + frm.datalen);
}

void DoRevisionConstantSet(uint8_t slot_id)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame data = {
		.slot_id = slot_id,
		.cmd = INTERNAL_CMD_REVISION_CONSTANT_SET,
		.datalen = 4,
	};
	memcpy(data.data, &TestData.revisionConstant[slot_id].UI8, 4);
	post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}


void DoRevisionConstantReq(uint8_t slot_id)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame data = {
		.slot_id = slot_id,
		.cmd = INTERNAL_CMD_REVISION_CONSTANT_REQ,
		.datalen = 0,
		.data = { 0 },
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}
