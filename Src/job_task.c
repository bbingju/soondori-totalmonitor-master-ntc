#include "job_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "0_Util.h"
#include <string.h>
#include "debug.h"


struct _job {
	JOB_TYPE_E type;
	union {
		struct external_frame_rx external_rx;
		struct external_frame_tx external_tx;
		struct internal_frame internal;
	};
};

static osMailQDef(job_q, 8, struct _job);
static osMailQId (job_q_id);

static void do_job(struct _job *job);
static void send_to_external(struct external_frame_tx *obj);
static void handle_from_external(struct external_frame_rx *obj);
static void request_to_internal(struct internal_frame *);
static void response_from_internal(struct internal_frame *);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

int post_job(JOB_TYPE_E type, void *data, size_t datalen)
{
	if (!data)
		return -1;

	struct _job *obj;
	obj = (struct _job *)osMailAlloc(job_q_id, osWaitForever);
	if (!obj) {
		DBG_LOG("error: mail alloc in %s\n", __func__);
		return -1;
	}
	obj->type = type;

	switch (type) {
	case JOB_TYPE_TO_INTERNAL:
	case JOB_TYPE_FROM_INTERNAL:
		memcpy(&obj->internal, data, datalen);
		break;
	case JOB_TYPE_FROM_EXTERNAL:
		memcpy(&obj->external_rx, data, datalen);
		break;
	case JOB_TYPE_TO_EXTERNAL:
		memcpy(&obj->external_tx, data, datalen);
		break;
	default:
		memcpy(&obj->internal, data, datalen);
		break;
	}

	osMailPut(job_q_id, obj);

	return 0;
}

void job_task(void const *arg)
{
	job_q_id = osMailCreate(osMailQ(job_q), NULL);

	while (1) {
		osEvent event = osMailGet(job_q_id, osWaitForever);
		struct _job *job = (struct _job *)event.value.p;
		do_job(job);
		osMailFree(job_q_id, job);
	}
}

static void do_job(struct _job *job)
{
	switch (job->type) {
	case JOB_TYPE_TO_EXTERNAL:
		send_to_external(&job->external_tx);
		break;
	case JOB_TYPE_FROM_EXTERNAL:
		handle_from_external(&job->external_rx);
		break;
	case JOB_TYPE_TO_INTERNAL:
		request_to_internal(&job->internal);
		break;
	case JOB_TYPE_FROM_INTERNAL:
		response_from_internal(&job->internal);
		break;

	default:
		return;
	}
}

extern void handle_rx_msg(struct external_frame_rx *received);
extern int ext_tx_completed;
extern UART_HandleTypeDef huart1;

static void send_to_external(struct external_frame_tx *ftx)
{
	static uint8_t tx_buffer[512] = {0};
	DBG_LOG("ext tx [%s::%s] (%d): ", ext_cmd_str(ftx->cmd),
		ext_option_str(ftx->cmd, ftx->option), ftx->len);
	/* DBG_DUMP(to_transmit_ext->raw, to_transmit_ext->bytes_to_transmit); */

	memset(tx_buffer, 0, sizeof(uint8_t) * 512);
	doMakeSend485Data(tx_buffer, ftx->cmd, ftx->option, ftx->data, ftx->data_padding_len, ftx->len - 20, ftx->len);
	/* int ftxsize = fill_external_tx_frame(tx_buffer, ftx->cmd, ftx->option, */
	/* 				ftx->ipaddr, ftx->datetime, ftx->data, ftx->len - 20); */

	DBG_DUMP(tx_buffer, ftx->len);
	if (ext_tx_completed) {
		ext_tx_completed = 0;
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1, tx_buffer, ftx->len);
	}
}

static void handle_from_external(struct external_frame_rx *obj)
{
	handle_rx_msg(obj);
}

static void request_to_internal(struct internal_frame *frm)
{
	size_t frame_size = 0;
	static uint8_t buffer[255 + 7] = {0};

	frame_size = fill_internal_frame(buffer, frm->slot_id, frm->cmd,
					frm->datalen, frm->data);
	DBG_LOG("JOB_TYPE_TO_INTERNAL:");
	DBG_DUMP(buffer, frame_size);

	HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
	HAL_UART_Transmit_DMA(&huart2, buffer, frame_size);

	/* struct internal_tx_msg_s *req = obj; */
	/* static uint8_t buffer[160] = {0}; */
	/* /\* DBG_LOG("JOB_TYPE_SEND_TO_INTERNAL:"); *\/ */
	/* /\* DBG_DUMP(req, sizeof(struct internal_tx_msg_s)); *\/ */

	/* memset(buffer, 0, sizeof(buffer)); */

	/* doMakeSendSlotData(buffer, req->id + 0x30, req->cmd, req->data,
	 * req->length, 14); */

	/* HAL_UART_Transmit_DMA(&huart2, buffer, 14); */
}

/* static void response_from_internal(struct internal_frame *frm) */
/* { */
/* 	DBG_LOG("JOB_TYPE_FROM_INTERNAL:"); */
/* 	DBG_LOG("slot id: %d, cmd: 0x%02X, datalen: %d", frm->slot_id, */
/* 		frm->cmd, frm->datalen); */
/* 	if (frm->datalen > 0) */
/* 		DBG_DUMP(frm->data, frm->datalen); */
/* 	else */
/* 		DBG_LOG("\n"); */

/* 	switch (frm->cmd) { */
/* 	case INTERNAL_CMD_BOARD_TYPE_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_RESET: */
/* 		break; */
/* 	case INTERNAL_CMD_SLOT_ID_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_TEMPERATURE_REQ: { */
/* 		/\* send_external_response(CMD_TEMP_TEST, *\/ */
/* 		/\* 		4, frm->datalen + 1) *\/ */
/* 		struct external_frame_tx data = { */
/* 			.cmd = CMD_TEMP_TEST, */
/* 			.option = 4/\* OP_TEMP_MAIN_INFO *\/, */
/* 			.len = sizeof(struct external_temp_data) + 20, */
/* 		}; */
/* 		struct external_temp_data *tdata = &data.temp_data; */
/* 		tdata->slot_id = frm->slot_id; */
/* 		memcpy(tdata->values, frm->data, sizeof(float) * 32); */
/* 		/\* data.ipaddr[0] = ; *\/ */
/* 		/\* datetime[0] = ; *\/ */
/* 		post_job(JOB_TYPE_TO_EXTERNAL, &data, data.len); */
/* 		break; */
/* 	} */
/* 	case INTERNAL_CMD_TEMPERATURE_STATE_REQ: { */
/* 		struct external_frame_tx data = { */
/* 			.cmd = CMD_TEMP_TEST, */
/* 			.option = 3, */
/* 			.len = sizeof(struct external_temp_state_data) + 20, */
/* 		}; */
/* 		struct external_temp_state_data *tdata = &data.temp_state_data; */
/* 		tdata->slot_id = frm->slot_id; */
/* 		memcpy(tdata->states, frm->data, sizeof(uint32_t) * 32); */
/* 		/\* data.ipaddr[0] = ; *\/ */
/* 		/\* datetime[0] = ; *\/ */
/* 		post_job(JOB_TYPE_TO_EXTERNAL, &data, data.len); */
/* 		break; */
/* 	} */
/* 	case INTERNAL_CMD_ADC_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_THRESHOLD_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_THRESHOLD_SET: */
/* 		break; */
/* 	case INTERNAL_CMD_RELAY_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_RELAY_SET: */
/* 		break; */
/* 	case INTERNAL_CMD_REVISION_CONSTANT_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_REVISION_CONSTANT_SET: */
/* 		break; */
/* 	case INTERNAL_CMD_REVISION_APPLY_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_REVISION_APPLY_SET: */
/* 		break; */
/* 	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_CAL: */
/* 		break; */
/* 	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_REQ: */
/* 		break; */
/* 	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_SET: */
/* 		break; */
/* 	} */
/* } */
