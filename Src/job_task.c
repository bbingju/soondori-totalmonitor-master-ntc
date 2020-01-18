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
	struct job *obj;

	if (!data)
		return -1;

	do {
		obj = (struct job *)osMailCAlloc(job_q_id, osWaitForever);
		__NOP();
	} while (obj == NULL);

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
	case JOB_TYPE_TO_EXTERNAL:
		_send_to_external(&job->d.external_tx);
		break;
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

static void _send_to_external(struct external_frame_tx *ftx)
{
	static uint8_t tx_buffer[512] = {0};
	/* DBG_LOG("ext tx [%s::%s] (%d): ", ext_cmd_str(ftx->cmd), */
	/* 	ext_option_str(ftx->cmd, ftx->option), ftx->len); */

	doMakeSend485Data(tx_buffer, ftx->cmd, ftx->option, ftx->data, ftx->data_padding_len, ftx->len - 20);
	/* int ftxsize = fill_external_tx_frame(tx_buffer, ftx->cmd, ftx->option, */
	/* 				ftx->ipaddr, ftx->datetime, ftx->data, ftx->len - 20); */

	/* DBG_DUMP(tx_buffer, ftx->len); */
	/* if (ext_tx_completed) { */
	while (!ext_tx_completed)
		__NOP();

	ext_tx_completed = 0;
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, tx_buffer, ftx->len);
	/* 	while (ext_tx_completed == 0) */
	/* 		__NOP(); */
	/* } */
}
