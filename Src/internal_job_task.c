#include "internal_job_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "internal_uart_task.h"
#include "0_Util.h"
#include <string.h>
#include "debug.h"

#define JOB_TANSACTION_TIMEOUT (50)

struct internal_job {
	INTERNAL_JOB_TYPE_E type;
	struct internal_frame int_frm;
};

static osMailQDef(internal_job_q, 6, struct internal_job);
static osMailQId (internal_job_q_id);

volatile int transaction_completed = 1;
static uint32_t transaction_timeout;

static void do_job(struct internal_job *);
static void request_to_internal(struct internal_frame *);

extern int int_tx_completed;
extern UART_HandleTypeDef huart2;

int post_internal_job(INTERNAL_JOB_TYPE_E type, void *data, size_t datalen)
{
	struct internal_job *obj;

	if (!data)
		return -1;

	do {
		obj = (struct internal_job *)osMailCAlloc(internal_job_q_id, osWaitForever);
		osDelay(1);
	} while (obj == NULL);

	obj->type = type;
	memcpy(&obj->int_frm, data, datalen);

	osMailPut(internal_job_q_id, obj);

	return 0;
}

void internal_job_task(void const *arg)
{
	internal_job_q_id = osMailCreate(osMailQ(internal_job_q), NULL);
	transaction_completed = 1;
	transaction_timeout = osKernelSysTick();

	while (1) {
		osEvent event = osMailGet(internal_job_q_id, osWaitForever);
		struct internal_job *job = (struct internal_job *)event.value.p;
		do_job(job);
		osMailFree(internal_job_q_id, job);
	}
}

static void do_job(struct internal_job *job)
{
	switch (job->type) {
	case INTERNAL_JOB_TYPE_TO:
		request_to_internal(&job->int_frm);
		break;
	default:
		return;
	}
}

static void request_to_internal(struct internal_frame *frm)
{
	size_t frame_size = 0;
	static uint8_t buffer[32 + 7] = {0};

	frame_size = fill_internal_frame(buffer, frm->slot_id, frm->cmd,
					frm->datalen, frm->data);

	DBG_LOG("int TX slot %d: %s - (datalen: %d) \n",
		frm->slot_id, int_cmd_str(frm->cmd), frm->datalen);
	/* DBG_DUMP(buffer, frame_size); */

	if (int_tx_completed && transaction_completed) {
		int_tx_completed = 0;
		HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
		/* HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_SET); */
		HAL_UART_Transmit_DMA(&huart2, buffer, frame_size);
		while (!int_tx_completed)
			__NOP();

		transaction_timeout = osKernelSysTick();
		transaction_completed = 0;
		__IO int count = 0;
		while (!transaction_completed) {
			if (osKernelSysTick() - transaction_timeout > JOB_TANSACTION_TIMEOUT) {
				DBG_LOG("internal job is canceled\r\n");
				transaction_completed = 1;
			}
			count++;
		}
		/* DBG_LOG("transaction_completed count: %d tick: %d\r\n", */
		/* count++, osKernelSysTick() - transaction_timeout); */
	}
}
