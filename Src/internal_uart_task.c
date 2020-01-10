#include "internal_uart_task.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "frame.h"
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

extern uint8_t crcErrorCount;
extern uint8_t startThreshold;
extern int transaction_completed;

static osMessageQDef(internal_rx_q, 48, uint32_t);
static osMessageQId(internal_rx_q_id);

void internal_rx_notify() { osMessagePut(internal_rx_q_id, 1, 0); }

int int_tx_completed = 1;

static uint8_t internal_rx_buffer[384 + 128];
static struct internal_frame int_frm_tx;

static void internal_rx_check(void);
static void parse_rx(const void *data, size_t len);

static void DoAnsBoardType(struct internal_frame *);
static void handle_threshold_req(struct internal_frame *);
static void handle_threshold_set(struct internal_frame *);
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
			SysProperties.slots[received->slot_id].inserted = TRUE;
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
		handle_threshold_req(received);
		break;
	case INTERNAL_CMD_THRESHOLD_SET:
		handle_threshold_set(received);
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
		SysProperties.slots[msg->slot_id].type = msg->data[0];
}

void send_slot_id_req(uint8_t id)
{
	HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
	DoRejectSlot();

    	struct internal_frame data = {
		.slot_id = id,
		.cmd = INTERNAL_CMD_SLOT_ID_REQ,
		.datalen = 0,
		.data = { 0 },
        };
        post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));

	/* uint8_t internal_id = id + 0x30; */
	/* request_internal(id, CMD_SLOT_ID_REQ, &internal_id, 1); */
}

static void handle_temperature(struct internal_frame *msg)
{
	if (msg) {
		memcpy(&TestData.temperatures[msg->slot_id], msg->temperatures.v, sizeof(float) * 32);
		/* uni4Byte *temp = &TestData.temperatures[msg->slot_id]; */
		/* for (int i = 0; i < 32; i++) */
		/* 	(temp + i)->Float = *((float *)&msg->data[i * 4]); */

		/* write log */
		if (SysProperties.last_slot_id == msg->slot_id)
			post_fs_job(FS_JOB_TYPE_SAVE_LOG);
	}
	/* send temperature values to the external */
	/* DoChannelValue(msg->slot_id); */

	/* DoIncSlotIdStep(readSlotNumber); */
}

static void handle_temerature_state(struct internal_frame *msg)
{
	if (msg) {
		struct internal_temp_state_data *sd = &msg->state_data;

		memcpy(&TestData.sensorState[msg->slot_id][0], sd, TEMP_STATE_DATA_LENGTH);
		memcpy(&TestData.sensorState[msg->slot_id][16], sd, TEMP_STATE_DATA_LENGTH);
	}
}

static void handle_threshold_req(struct internal_frame *msg)
{
	if (!msg)
		return;

	memcpy(&TestData.thresholds[msg->slot_id], &msg->thresholds, sizeof(float) * 32);

	uint8_t thresholdData[130] = {0};
	thresholdData[0] = msg->slot_id;
	memcpy(&thresholdData[1], &TestData.thresholds, sizeof(float) * 32);
	send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ,
			thresholdData, 129, 132, 152);
}

static void handle_threshold_set(struct internal_frame *msg)
{
	if (!msg)
		return;

	memcpy(&TestData.thresholds[msg->slot_id], &msg->thresholds, sizeof(float) * 32);
        /* float *thresholds = &TestData.thresholds[msg->slot_id]; */
        /* for (int i = 0; i < 32; i++) { */
	/* 	*(thresholds + i) = msg->thresholds.v[i]; */
        /* } */

	uint8_t data[130] = {0};
	data[0] = msg->slot_id;
	memcpy(&data[1], &TestData.thresholds, sizeof(float) * 32);
	send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_SET, data, 129,
			132, 152);
}

static void DoAnsRevisionApplySet(struct internal_frame *msg)
{
	if (!msg)
		return;

	TestData.revisionApply[msg->slot_id] = msg->data[0];

	static uint8_t revision_apply[2] = {0};
	revision_apply[0] = msg->slot_id;
	revision_apply[1] = TestData.revisionApply[msg->slot_id];
	send_external_response(CMD_REVISION, OP_REVISION_APPLY_SET, revision_apply, 2, 12, 32);
}

static void DoAnsRevisionApplyReq(struct internal_frame *msg)
{
	if (!msg)
		return;

        TestData.revisionApply[msg->slot_id] = msg->data[0];

	uint8_t data[2];
	data[0] = msg->slot_id;
	data[1] = TestData.revisionApply[msg->slot_id];

	send_external_response(CMD_REVISION, OP_REVISION_APPLY_REQ, data, 2, 12, 32);
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
	send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_SET, data, 6, 12, 32);
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
	send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_REQ, data, 6, 12, 32);
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
	send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL,
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

	send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET,
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
	send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ,
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

	send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ,
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

void DoThresholdSet(struct slot_properties_s *slot, uint8_t channel, float threshold)
{
	if (slot && slot->inserted) {
		int_frm_tx.slot_id = slot->id;
		int_frm_tx.cmd = INTERNAL_CMD_THRESHOLD_SET;
		int_frm_tx.datalen = 5;
		int_frm_tx.data[0] = channel;
		*((float *) &int_frm_tx.data[1]) = threshold;

		post_internal_job(INTERNAL_JOB_TYPE_TO, &int_frm_tx, sizeof(int_frm_tx));
	}
}

void DoThresholdReq(struct slot_properties_s *slot)
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

/*********************************************************************
 *       doReqSlotID
 *       SLOT 각 보드의 ID 를 설정
 *   slotNumber : SLOT 지정
 **********************************************************************/
void DoReqSlotID(uint8_t slotNumber)
{
	HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
	DoRejectSlot();

	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_SLOT_ID_REQ,
		.datalen = 0,
		.data = { 0 },
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

/*********************************************************************
 *       doReqTemperature
 *       슬롯에 센서 온도를 요청 함, 전송 받는 값은 flot(4byte)으로 한다.
 *   slotNumber : SLOT 지정
 **********************************************************************/
void DoReqTemperature(struct slot_properties_s *slot)
{
	if (slot && slot->inserted) {
		/* HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET); */
		/* request_internal(slot->id, CMD_TEMP_REQ, NULL, 0); */
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_TEMPERATURE_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

void DoReqTemperatureState(struct slot_properties_s *slot)
{
	if (slot && slot->inserted) {
		/* HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET); */
		/* request_internal(slot->id, CMD_TEMP_STATE_REQ, NULL, 0); */
		struct internal_frame data = {
			.slot_id = slot->id,
			.cmd = INTERNAL_CMD_TEMPERATURE_STATE_REQ,
			.datalen = 0,
			.data = { 0 },
		};
		post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
	}
}

 // slot 번호 전달 , 0: 측정온도 모드, 1: 보정온도 모드
void DoRevisionApplySet(uint8_t slotNumber, uint8_t const mode)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame frame = {
		.cmd = INTERNAL_CMD_REVISION_APPLY_SET,
		.datalen = 1,
	};

	if (slotNumber != 0xFF) {
		frame.slot_id = slotNumber;
		frame.revision_apply.enabled = mode;
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frame, 3 + frame.datalen);
	} else {
		frame.revision_apply.enabled = mode;

		struct slot_properties_s *slot;
		for (int i = 0; i < MAX_SLOT_NUM; i++) {
			slot = &SysProperties.slots[i];
			if (slot->inserted) {
				frame.slot_id = slot->id;
				post_internal_job(INTERNAL_JOB_TYPE_TO, &frame, 3 + frame.datalen);
				osDelay(1);
			}
		}
	}
}

void DoRevisionConstantSet(uint8_t slotNumber)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_REVISION_CONSTANT_SET,
		.datalen = 4,
	};
	memcpy(data.data, &TestData.revisionConstant[slotNumber].UI8, 4);
	post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

void DoRevisionApplyReq(uint8_t slotNumber)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame frame = {
		.cmd = INTERNAL_CMD_REVISION_APPLY_REQ,
		.datalen = 0,
		.data = { 0 },
	};

	if (slotNumber != 0xFF) {
		frame.slot_id = slotNumber;
		post_internal_job(INTERNAL_JOB_TYPE_TO, &frame, 3 + frame.datalen);
	} else {
		struct slot_properties_s *slot;
		for (int i = 0; i < MAX_SLOT_NUM; i++) {
			slot = &SysProperties.slots[i];
			if (slot->inserted) {
				frame.slot_id = slot->id;
				post_internal_job(INTERNAL_JOB_TYPE_TO, &frame, 3 + frame.datalen);
				osDelay(1);
			}
		}
	}
}

void DoRevisionConstantReq(uint8_t slotNumber)
{
	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

	struct internal_frame data = {
		.slot_id = slotNumber,
		.cmd = INTERNAL_CMD_REVISION_CONSTANT_REQ,
		.datalen = 0,
		.data = { 0 },
	};
	post_internal_job(INTERNAL_JOB_TYPE_TO, &data, sizeof(data));
}

/*********************************************************************
 *       doIncSlotIdStep
 *       슬롯이 4개(차후 6개로 증가 예정)를 순차적으로 요청 하시 위한 함수
 *   slotNumber : SLOT 지정
 **********************************************************************/
/* void DoIncSlotIdStep(uint8_t slotNumber) */
/* { */
/*     uint8_t ct = 0; */

/*     switch (SysProperties.InterfaceStep) { */
/*     case STEP_SLOT_ID: */
/*         SendSlotNumber = slotNumber + 1; */
/*         if (SendSlotNumber > 3) { */
/*             SendSlotNumber = 0; */
/*             SysProperties.InterfaceStep = STEP_READ_THRESHOLD; */
/*             /\* RxQueue_Clear(&RxQueue); *\/ */

/*             bool system_reset_needed = true; */
/*             for (int i = 0; i < 4; i++) { */
/*                 if (SysProperties.slots[i].inserted) { */
/*                     system_reset_needed = false; */
/*                     break; */
/*                 } */
/*             } */
/*             if (system_reset_needed) */
/*                 HAL_NVIC_SystemReset(); */
/*         } */
/*         break; */
/*     case STEP_READ_THRESHOLD: //각 슬롯의 경고 온도 값을 불러 온다. */
/*         do { */
/*             SendSlotNumber = ++slotNumber; */
/*             if (SendSlotNumber > 3) { */
/*                 SendSlotNumber = 0; */
/*                 SysProperties.InterfaceStep = STEP_TEMP_READ; */
/*                 /\* RxQueue_Clear(&RxQueue); *\/ */
/*             } */
/*             ct++; */
/*             if (ct > 3) */
/*                 break; */
/*         } while (!SysProperties.slots[SendSlotNumber].inserted); */
/*         break; */
/*     case STEP_TEMP_READ: */
/*         do { */
/*             SendSlotNumber = ++slotNumber; */
/*             if (SendSlotNumber > 3) { */
/*                 SendSlotNumber = 0; */
/*                 startThreshold = FALSE; */
/*             } */
/*             ct++; */
/*             if (ct > 3) */
/*                 break; */
/*         } while (!SysProperties.slots[SendSlotNumber].inserted); */
/*         break; */
/*     } */
/* } */

/*********************************************************************
**********************************************************************
*
*       Rx 관련 함수
*
*   doAns 으로 시작 하는 함수 : 요청한 응답이 있거나, 설정 완료에 대한 응답
**********************************************************************
**********************************************************************/

/*********************************************************************
 *       doSlotJumpFunction
 *       Rx Data 를 Parsing 하기 위한 분기 함수
 **********************************************************************/
/* void DoSlotJumpFunction(void) */
/* { */
/*     if(RxSlotData[0] == CMD_STX) */
/*     { */
/*         //if(RxSlotData[11] == CMD_ETX) */
/*         if(RxReadCount == 11) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_BOARD_TYPE: */
/*                 DoAnsBoardType(); */
/*                 break; */
/*             case CMD_BOARD_EN_REQ: */
/*                 break; */
/*             case CMD_BOARD_EN_SET: */
/*                 break; */
/*             case CMD_SLOT_ID_REQ: */
/*                 DoAnsReqSlotID(); */
/*                 break; */
/*             case CMD_HW_VER: */
/*                 break; */
/*             case CMD_FW_VER: */
/*                 break; */
/*             case CMD_UUID_REQ: */
/*                 break; */
/*             case CMD_ADC_REQ: */
/*                 break; */
/*             case CMD_RELAY_REQ: */
/*                 break; */
/*             case CMD_RELAY_SET: */
/*                 break; */
/*             case CMD_REVISION_APPLY_SET: */
/*                 DoAnsRevisionApplySet(); */
/*                 break; */
/*             case CMD_REVISION_CONSTANT_SET: */
/*                 DoAnsRevisionConstantSet(); */
/*                 break; */
/*             case CMD_REVISION_APPLY_REQ: */
/*                 DoAnsRevisionApplyReq(); */
/*                 break; */
/*             case CMD_REVISION_CONSTANT_REQ: */
/*                 DoAnsRevisionConstantReq(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CONSTANT_SET: */
/*                 DoAnsCalibrationNTCConstantSet(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CONSTANT_REQ: */
/*                 DoAnsCalibrationNTCConstantReq(); */
/*                 break; */
/*             } */
/*         } */
/*         //else if(RxSlotData[37] == CMD_ETX) */
/*         else if(RxReadCount == 37) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_TEMP_STATE_REQ: */
/*                 DoAnsTemperatureState(); */
/*                 break; */
/*             } */
/*         } */
/*         //else if(RxSlotData[133] == CMD_ETX) */
/*         else if(RxReadCount == 133) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_TEMP_REQ: */
/*                 DoAnsTemperature(); */
/*                 break; */
/*             case CMD_THRESHOLD_REQ: */
/*                 DoAnsThresholdReq(); */
/*                 break; */
/*             case CMD_THRESHOLD_SET: */
/*                 DoAnsThresholdSet(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CON_TABLE_CAL: */
/*                 DoAnsCalibrationNTCTableCal(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CON_TABLE_REQ: */
/*                 DoAnsCalibrationNTCTableReq(); */
/*                 break; */
/*             } */
/*         } */
/*     } */
/* } */

/*********************************************************************
 *       doAnsBoardType
 *       보드 타입 응답 함수
 **********************************************************************/
/* void DoAnsBoardType(struct internal_frame *msg) */
/* { */
/*   if (msg) */
/*     SysProperties.slotType[msg->slot_id] = msg->data[0]; */
/* } */

/*********************************************************************
 *       doAnsReqSlotID
 *       슬롯 id 설정 확인 함수
 **********************************************************************/
/* void DoAnsReqSlotID(void) */
/* { */
/*     noReturnSendCt = 0; */

/*     if(SendSlotNumber == (RxSlotData[1] - '0')) */
/*     { */
/*         SysProperties.slotInsert[RxSlotData[1] - '0'] = TRUE; */
/*         DoIncSlotIdStep(SendSlotNumber); */
/*     } */
/* } */

/*********************************************************************
 *       doAnsTemperature
 *       온도 요청에 대한 응답, 회신 되는 응답은 flot임.
 **********************************************************************/
/* void DoAnsTemperature(void) */
/* { */
/*     uint8_t i; */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(i = 0; i < 32; i++) */
/*         { */
/*             HAL_Delay(1); */
/*             TestData.temperature[readSlotNumber][i].UI8[0] = RxSlotData[i * 4
 * + 3]; */
/*             TestData.temperature[readSlotNumber][i].UI8[1] = RxSlotData[i * 4
 * + 4]; */
/*             TestData.temperature[readSlotNumber][i].UI8[2] = RxSlotData[i * 4
 * + 5]; */
/*             TestData.temperature[readSlotNumber][i].UI8[3] = RxSlotData[i * 4
 * + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     DoIncSlotIdStep(readSlotNumber); */
/* } */

/* void DoAnsTemperatureState(void) */
/* { */
/*     uint8_t count = 0; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     for(count = 0; count < 32; count++) */
/*     { */
/*         TestData.sensorState[readSlotNumber][count] =
 * (LED_DIPLAY_MODE)RxSlotData[count + 3]; */
/*     } */
/* } */

/* void DoAnsThresholdReq(void) */
/* { */
/*     uni2Byte        crc; */
/*     uint8_t         thresholdData[130]; */

/*     noReturnSendCt = 0; */
/*     thresholdData[0] = RxSlotData[1] - '0'; */
/*     readSlotNumber = thresholdData[0]; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for (int inc = 0; inc < 32; inc++) */
/*         { */
/*             TestData.threshold[readSlotNumber][inc].UI8[0] = RxSlotData[inc *
 * 4 + 3]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[1] = RxSlotData[inc *
 * 4 + 4]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[2] = RxSlotData[inc *
 * 4 + 5]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[3] = RxSlotData[inc *
 * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     if(startThreshold != TRUE)      //초기화 하는 동안은 486 전송 하지
 * 않는다, 초기화 중일때 startThreshold == TRUE 임. */
/*     { */
/*         memcpy(&thresholdData[1],
 * &TestData.threshold[thresholdData[0]][0].UI8[0], 128); */
/*         doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP,
 * OP_WARNING_TEMP_REQ, &thresholdData[0], 129, 132, 152); */
/*         SendUart485String(tx485DataDMA, 152); */
/*     } */

/*     if(startThreshold == TRUE) */
/*     { */
/*         DoIncSlotIdStep(thresholdData[0]); */
/*     } */
/* } */

/* void DoAnsThresholdSet(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         thresholdData[130]; */

/*     noReturnSendCt = 0; */
/*     thresholdData[0] = RxSlotData[1] - '0'; */
/*     readSlotNumber = thresholdData[0]; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             TestData.threshold[readSlotNumber][inc].UI8[0] = RxSlotData[inc *
 * 4 + 3]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[1] = RxSlotData[inc *
 * 4 + 4]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[2] = RxSlotData[inc *
 * 4 + 5]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[3] = RxSlotData[inc *
 * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     memcpy(&thresholdData[1], &TestData.threshold[readSlotNumber][0].UI8[0],
 * 128); */
/*     doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP, OP_WARNING_TEMP_SET,
 * &thresholdData[0], 129, 132, 152); */
/*     /\* DBG_LOG("%s: ", __func__); *\/ */
/*     /\* print_bytes(tx485DataDMA, 152); *\/ */
/*     SendUart485String(tx485DataDMA, 152); */
/* } */

/* void DoAnsCalibrationNTCTableCal(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         calData[130]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             osDelay(1); */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[0] =
 * RxSlotData[inc * 4 + 3]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[1] =
 * RxSlotData[inc * 4 + 4]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[2] =
 * RxSlotData[inc * 4 + 5]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[3] =
 * RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     calData[0] = readSlotNumber; */
/*     memcpy((void*)&calData[1],(void*)&TestData.ntcCalibrationTable[readSlotNumber][0].UI8[0],
 * 128); */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL,
 * calData, 129, 132, 152); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
 * OP_CALIBRATION_NTC_CON_TABLE_CAL, calData, 129, 132, 152); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 152); *\/ */
/* } */

/* void DoAnsRevisionApplySet(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[2]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionApply[readSlotNumber] = RxSlotData[3]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionApply[readSlotNumber]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_APPLY_SET, revData, 2,
 * 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_APPLY_SET,
 * revData, 2, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionConstantSet(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[5]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionConstant[readSlotNumber].UI8[0] = RxSlotData[3]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[1] = RxSlotData[4]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[2] = RxSlotData[5]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[3] = RxSlotData[6]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionConstant[readSlotNumber].UI8[0]; */
/*     revData[2] = TestData.revisionConstant[readSlotNumber].UI8[1]; */
/*     revData[3] = TestData.revisionConstant[readSlotNumber].UI8[2]; */
/*     revData[4] = TestData.revisionConstant[readSlotNumber].UI8[3]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_SET, revData,
 * 6, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION,
 * OP_REVISION_CONSTANT_SET, revData, 6, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionApplyReq(void) */
/* { */

/*     uni2Byte crc; */
/*     uint8_t  revData[2]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionApply[readSlotNumber] = RxSlotData[3]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionApply[readSlotNumber]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_APPLY_REQ, revData, 2,
 * 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_APPLY_REQ,
 * revData, 2, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionConstantReq(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[5]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionConstant[readSlotNumber].UI8[0] = RxSlotData[3]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[1] = RxSlotData[4]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[2] = RxSlotData[5]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[3] = RxSlotData[6]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionConstant[readSlotNumber].UI8[0]; */
/*     revData[2] = TestData.revisionConstant[readSlotNumber].UI8[1]; */
/*     revData[3] = TestData.revisionConstant[readSlotNumber].UI8[2]; */
/*     revData[4] = TestData.revisionConstant[readSlotNumber].UI8[3]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_REQ, revData,
 * 6, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION,
 * OP_REVISION_CONSTANT_REQ, revData, 6, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsCalibrationNTCConstantSet(void) */
/* { */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.ntcCalibrationConst.UI8[0] = RxSlotData[3]; */
/*         TestData.ntcCalibrationConst.UI8[1] = RxSlotData[4]; */
/*         TestData.ntcCalibrationConst.UI8[2] = RxSlotData[5]; */
/*         TestData.ntcCalibrationConst.UI8[3] = RxSlotData[6]; */

/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET,
 */
/*                            &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32);
 */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
 * OP_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConst.UI8[0], 4, 12,
 * 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsCalibrationNTCTableReq(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         calData[130]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             osDelay(1); */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[0] =
 * RxSlotData[inc * 4 + 3]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[1] =
 * RxSlotData[inc * 4 + 4]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[2] =
 * RxSlotData[inc * 4 + 5]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[3] =
 * RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     calData[0] = readSlotNumber; */
/*     memcpy((void*)&calData[1],
 * (void*)&TestData.ntcCalibrationTable[readSlotNumber][0].UI8[0], 128); */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ,
 * calData, 129, 132, 152); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
 * OP_CALIBRATION_NTC_CON_TABLE_REQ, calData, 129, 132, 152); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 152); *\/ */
/* } */

/* void DoAnsCalibrationNTCConstantReq(void) */
/* { */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.ntcCalibrationConst.UI8[0] = RxSlotData[3]; */
/*         TestData.ntcCalibrationConst.UI8[1] = RxSlotData[4]; */
/*         TestData.ntcCalibrationConst.UI8[2] = RxSlotData[5]; */
/*         TestData.ntcCalibrationConst.UI8[3] = RxSlotData[6]; */

/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ,
 * &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
 * OP_CALIBRATION_NTC_CONSTANT_REQ, &TestData.ntcCalibrationConst.UI8[0], 4, 12,
 * 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */
