#include "job_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "internal_uart_task.h"
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

static osMailQDef(job_q, 12, struct _job);
static osMailQId (job_q_id);

static int is_transaction;
static uint32_t transaction_timeout;

static void do_job(struct _job *job);
static void send_to_external(struct external_frame_tx *obj);
static void handle_from_external(struct external_frame_rx *obj);
static void request_to_internal(struct internal_frame *);

extern void handle_rx_msg(struct external_frame_rx *received);
extern int ext_tx_completed;
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
	is_transaction = 0;
	transaction_timeout = osKernelSysTick();

	while (1) {
		if (is_transaction &&
			osKernelSysTick() - transaction_timeout > 300) {
			/* Cancel the internal transction */
			is_transaction = 0;
		}

		osEvent event = osMailGet(job_q_id, osWaitForever);
		struct _job *job = (struct _job *)event.value.p;

		if (job->type == JOB_TYPE_TO_INTERNAL) {
			if (!is_transaction) {
				transaction_timeout = osKernelSysTick();
				is_transaction = 1;
			} else {
				osMailPut(job_q_id, job);
				continue;
			}
		} else if (job->type == JOB_TYPE_FROM_INTERNAL) {
			is_transaction = 0;
		}
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

static void send_to_external(struct external_frame_tx *ftx)
{
	static uint8_t tx_buffer[512] = {0};
	DBG_LOG("ext tx [%s::%s] (%d): ", ext_cmd_str(ftx->cmd),
		ext_option_str(ftx->cmd, ftx->option), ftx->len);

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
	/* HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_SET); */

	HAL_UART_Transmit_DMA(&huart2, buffer, frame_size);
}
