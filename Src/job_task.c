#include "job_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "internal_uart_task.h"
#include "external_uart_task.h"
#include "0_Util.h"
#include <string.h>
#include "debug.h"

#define JOB_TANSACTION_TIMEOUT 300

struct job {
	JOB_TYPE_E type;
	union {
		struct external_frame_rx external_rx;
		struct external_frame_tx external_tx;
		struct internal_frame internal;
	} d;
};

static osMailQDef(job_q, 24, struct job);
static osMailQId (job_q_id);

static void do_job(struct job *job);
static void _send_to_external(struct external_frame_tx *obj);

extern int ext_tx_completed;
extern UART_HandleTypeDef huart1;

int post_job(JOB_TYPE_E type, void *data, size_t datalen)
{
	if (!data)
		return -1;

	struct job *obj = (struct job *)osMailCAlloc(job_q_id, osWaitForever);
	obj->type = type;
	memcpy(&obj->d, data, datalen);

	osMailPut(job_q_id, obj);

	return 0;
}

void job_task(void const *arg)
{
	job_q_id = osMailCreate(osMailQ(job_q), NULL);


	while (1) {
		osEvent event = osMailGet(job_q_id, osWaitForever);
		struct job *job = (struct job *)event.value.p;
		do_job(job);
		osMailFree(job_q_id, job);
	}
}

static void do_job(struct job *job)
{
	switch (job->type) {
	case JOB_TYPE_TO_EXTERNAL: {
		/* RTC_DateTypeDef *d = &SysTime.Date; */
		/* RTC_TimeTypeDef *t = &SysTime.Time; */

		/* DBG_LOG("%02d%02d%02d_%02d%02d%02d\r\n", d->Year, d->Month, d->Date, */
		/* 	t->Hours, t->Minutes, t->Seconds); */

		_send_to_external(&job->d.external_tx);
		break;
	}

	case JOB_TYPE_FROM_EXTERNAL:
		receive_from_external(&job->d.external_rx);
		break;
	case JOB_TYPE_FROM_INTERNAL:
		response_from_internal(&job->d.internal);
		break;
	default:
		return;
	}
}

const char *ext_cmd_str(uint8_t cmd)
{
    switch (cmd) {
    case CMD_TEMP_TEST:
        return "CMD_TEMP_TEST";
    case CMD_WARNING:
        return "CMD_WARNING";
    case CMD_COMPENSATE:
        return "CMD_COMPENSATE";
    case CMD_CORRECTION:
        return "CMD_CORRECTION";
    case CMD_SD_CARD:
        return "CMD_SD_CARD";
    case CMD_SLOT:
        return "CMD_SLOT";
    case CMD_TIME:
        return "CMD_TIME";
    }

    return "";
}

const char *ext_option_str(uint8_t cmd, uint8_t option)
{
	switch (cmd) {
	case CMD_TEMP_TEST:
		switch (option) {
		case OP_TEMP_START_RX:
			return "OP_TEMP_START_RX";
		case OP_TEMP_STOP:
			return "OP_TEMP_STOP";
		case OP_TEMP_SAMPLE_RATE:
			return "OP_TEMP_SAMPLE_RATE";
		case OP_TEMP_SLOT_INFO:
			return "OP_TEMP_SLOT_INFO";
		case OP_TEMP_CHANNEL_INFO:
			return "OP_TEMP_CHANNEL_INFO";
		case OP_TEMP_CHANNEL_VALUE:
			return "OP_TEMP_CHANNEL_VALUE";
		}
		break;
	case CMD_WARNING:
		switch (option) {
		case OP_WARNING_THRESHOLD_SET:
			return "OP_WARNING_THRESHOLD_SET";
		case OP_WARNING_THRESHOLD_REQ:
			return "OP_WARNING_THRESHOLD_REQ";
		}
		break;
	case CMD_COMPENSATE:
		switch (option) {
		case OP_COMPENSATED_APPLY_SET:
			return "OP_COMPENSATED_APPLY_SET";
		case OP_COMPENSATED_CONSTANT_SET:
			return "OP_COMPENSATED_CONSTANT_SET";
		case OP_COMPENSATED_APPLY_REQ:
			return "OP_COMPENSATED_APPLY_REQ";
		case OP_COMPENSATED_CONSTANT_REQ:
			return "OP_COMPENSATED_CONSTANT_REQ";
		}
		break;
	case CMD_CORRECTION:
		switch (option) {
		case OP_CORRECTION_RTD_CONSTANT_SET:
			return "OP_CORRECTION_RTD_CONSTANT_SET";
		case OP_CORRECTION_NTC_CON_TABLE_CAL:
			return "OP_CORRECTION_NTC_CON_TABLE_CAL";
		case OP_CORRECTION_NTC_CONSTANT_SET:
			return "OP_CORRECTION_NTC_CONSTANT_SET";
		case OP_CORRECTION_RTD_CONSTANT_REQ:
			return "OP_CORRECTION_RTD_CONSTANT_REQ";
		case OP_CORRECTION_NTC_CON_TABLE_REQ:
			return "OP_CORRECTION_NTC_CON_TABLE_REQ";
		case OP_CORRECTION_NTC_CONSTANT_REQ:
			return "OP_CORRECTION_NTC_CONSTANT_REQ";
		}
		break;
	case CMD_SD_CARD:
		switch (option) {
		case OP_SDCARD_LIST_START:
			return "OP_SDCARD_LIST_START";
		case OP_SDCARD_LIST_BODY:
			return "OP_SDCARD_LIST_BODY";
		case OP_SDCARD_LIST_END:
			return "OP_SDCARD_LIST_END";
			/* case OP_SDCARD_LIST: */
			/*     return "OP_SDCARD_LIST"; */
			/* case OP_SDCARD_DOWNLOAD: */
			/*     return "OP_SDCARD_DOWNLOAD"; */
		case OP_SDCARD_DOWNLOAD_HEADER:
			return "OP_SDCARD_DOWNLOAD_HEADER";
		case OP_SDCARD_DOWNLOAD_BODY:
			return "OP_SDCARD_DOWNLOAD_BODY";
		case OP_SDCARD_DOWNLOAD_END:
			return "OP_SDCARD_DOWNLOAD_END";
		case OP_SDCARD_DELETE:
			return "OP_SDCARD_DELETE";
		case OP_SDCARD_FORMAT:
			return "OP_SDCARD_FORMAT";
		case OP_SDCARD_ERROR:
			return "OP_SDCARD_ERROR";
		}
		break;
	case CMD_SLOT:
		switch (option) {
		case OP_SLOT_SET:
			return "OP_SLOT_SET";
		case OP_SLOT_REQ:
			return "OP_SLOT_REQ";
		}
		break;
	case CMD_TIME:
		switch (option) {
		case OP_TIME_SET:
			return "OP_TIME_SET";
		case OP_TIME_REQ:
			return "OP_TIME_REQ";
		}
		break;
	}
	return "";
}

static void _send_to_external(struct external_frame_tx *ftx)
{
	static uint8_t tx_buffer[512] = {0};
	/* DBG_LOG("ext tx [%s::%s] (%d): ", ext_cmd_str(ftx->cmd), */
	/* 	ext_option_str(ftx->cmd, ftx->option), ftx->len); */

	while (!ext_tx_completed)
		__NOP();

	/* doMakeSend485Data(tx_buffer, ftx->cmd, ftx->option, ftx->data, ftx->data_padding_len, ftx->len - 20); */
	int ftxsize = fill_external_tx_frame(tx_buffer, ftx->cmd, ftx->option, ftx->data, ftx->data_padding_len);

	/* DBG_DUMP(tx_buffer, ftx->len); */

	ext_tx_completed = 0;
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, tx_buffer, ftxsize);
	/* HAL_UART_Transmit_DMA(&huart1, tx_buffer, ftx->len); */
	while (ext_tx_completed == 0)
		__NOP();
}
