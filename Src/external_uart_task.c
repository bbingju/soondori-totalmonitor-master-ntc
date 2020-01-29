#include "external_uart_task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_dma.h"
/* #include "stm32f4xx_ll_dma.h" */
#include "internal_uart_task.h"
#include "app_task.h"
#include "app_ctx.h"
#include "0_soonFlashMemory.h"
#include "0_SdCard.h"
#include "protocol.h"
#include "job_task.h"
#include "fs_task.h"
#include "debug.h"

#include <stdint.h>
#include <string.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

extern app_ctx_t ctx;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
__IO int ext_tx_completed = 1;

static void parse_rx(const void *data, size_t len);

static void cmd_warning__SET_THRESHOLD(struct external_frame_rx *);
static void cmd_warning__GET_THRESHOLD(struct external_frame_rx *);
static void cmd_warning__SET_VARIATION(struct external_frame_rx *);
static void cmd_warning__GET_VARIATION(struct external_frame_rx *);
static void cmd_compensated__SET(struct external_frame_rx *);
static void cmd_compensated__GET(struct external_frame_rx *);
static void cmd_compensated__SET_CONTACT_CONST(struct external_frame_rx *); /* NOT USED */
static void cmd_compensated__GET_CONTACT_CONST(struct external_frame_rx *); /* NOT USED */
static void cmd_compensated__SET_TR_CONST(struct external_frame_rx *);
static void cmd_compensated__GET_TR_CONST(struct external_frame_rx *);
static void cmd_correction__SET_RTD_CONST(struct external_frame_rx *);
static void cmd_correction__GET_RTD_CONST(struct external_frame_rx *);
static void cmd_correction__CAL_NTC_TBL(struct external_frame_rx *);
static void cmd_correction__GET_NTC_TBL(struct external_frame_rx *);
static void CmdCalibrationNTCConstantSet(struct external_frame_rx *); /* NOT USED */
static void CmdCalibrationNTCConstantReq(struct external_frame_rx *); /* NOT USED */
static void cmd_time__SET_TIME(struct external_frame_rx *);
static void cmd_time__GET_TIME(struct external_frame_rx *);
static void cmd_sys__GET_FW_VER(struct external_frame_rx *);
static void cmd_surveillance__SET_INTERVAL(struct external_frame_rx *);
static void cmd_sd__GET_ERROR(struct external_frame_rx *);

/*********************************************************************
*	Private variables
**********************************************************************/
static osMessageQDef(ext_rx_q, 24, uint32_t);
static osMessageQId(ext_rx_q_id);

uint8_t ext_rx_buffer[1024];

void ext_rx_notify() { osMessagePut(ext_rx_q_id, 1, 0); }

/**
 * @brief  Check for new data received with DMA
 */
static void ext_rx_check(void)
{
	static size_t old_pos;
	size_t pos;

	/* Calculate current position in buffer */
	pos = ARRAY_LEN(ext_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	/* pos = ARRAY_LEN(ext_rx_buffer) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5); */
	if (pos != old_pos) {    /* Check change in received data */
		if (pos > old_pos) { /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			parse_rx(&ext_rx_buffer[old_pos], pos - old_pos);
		} else {
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			parse_rx(&ext_rx_buffer[old_pos], ARRAY_LEN(ext_rx_buffer) - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				parse_rx(&ext_rx_buffer[0], pos);
			}
		}
	} else {

		huart1.Instance = USART1;
		huart1.Init.BaudRate = 921600;
		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.Mode = UART_MODE_TX_RX;
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		USART_TypeDef *regs = huart1.Instance;
		SET_BIT(regs->CR1, USART_CR1_IDLEIE);

		hdma_usart1_rx.Instance = DMA2_Stream5;
		hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		HAL_UART_Receive_DMA(&huart1, ext_rx_buffer, ARRAY_LEN(ext_rx_buffer));
	}
	old_pos = pos; /* Save current position as old */

	/* Check and manually update if we reached end of buffer */
	if (old_pos == ARRAY_LEN(ext_rx_buffer)) {
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
	static  struct external_frame_rx frm = {0};

	for (int i = 0; i < len; i++) {
		if (parse_external_rx_frame(&frm, data + i)) {
			post_job(JOB_TYPE_FROM_EXTERNAL, &frm, sizeof(frm));
		}
	}
}

extern void send_to_external__BOARD_INFO();

void receive_from_external(struct external_frame_rx *received)
{
	/* DBG_LOG("ext rx [%s::%s]: ", ext_cmd_str(received->cmd), */
	/* 	ext_option_str(received->cmd, received->option)); */
	/* DBG_DUMP(received->data, 22); */

	switch (received->cmd) {
	case CMD_TEMP_TEST:
		switch(received->option) {
		case OP_TEMP_START_RX: {
			start_temperature_measuring();
			SysProperties.start_flag = TRUE;
			break;
		}
		case OP_TEMP_STOP:
			SysProperties.start_flag = FALSE;
			stop_temperature_measuring();
			break;
		case OP_TEMP_SAMPLE_RATE:
			cmd_surveillance__SET_INTERVAL(received);
			break;
		}
		break;
	case CMD_WARNING:
		switch (received->option) {
		case OP_WARNING_THRESHOLD_SET:
			cmd_warning__SET_THRESHOLD(received);
			break;
		case OP_WARNING_THRESHOLD_REQ:
			cmd_warning__GET_THRESHOLD(received);
			break;
		case OP_WARNING_VARIATION_SET:
			cmd_warning__SET_VARIATION(received);
			break;
		case OP_WARNING_VARIATION_REQ:
			cmd_warning__GET_VARIATION(received);
			break;
		}
		break;
	case CMD_COMPENSATE:
		switch (received->option) {
		case OP_COMPENSATED_APPLY_SET:
			cmd_compensated__SET(received);
			break;
		case OP_COMPENSATED_APPLY_REQ:
			cmd_compensated__GET(received);
			break;
		case OP_COMPENSATED_CONSTANT_SET:
			cmd_compensated__SET_CONTACT_CONST(received);
			break;
		case OP_COMPENSATED_CONSTANT_REQ:
			cmd_compensated__GET_CONTACT_CONST(received);
			break;
		case OP_COMPENSATED_TR_CONST_SET:
			cmd_compensated__SET_TR_CONST(received);
			break;
		case OP_COMPENSATED_TR_CONST_REQ:
			cmd_compensated__GET_TR_CONST(received);
			break;
		}
		break;
	case CMD_CORRECTION:
		switch (received->option) {
		case OP_CORRECTION_RTD_CONSTANT_SET:
			cmd_correction__SET_RTD_CONST(received);
			break;
		case OP_CORRECTION_NTC_CON_TABLE_CAL:
			cmd_correction__CAL_NTC_TBL(received);
			break;
		case OP_CORRECTION_NTC_CONSTANT_SET:
			CmdCalibrationNTCConstantSet(received);
			break;
		case OP_CORRECTION_RTD_CONSTANT_REQ:
			cmd_correction__GET_RTD_CONST(received);
			break;
		case OP_CORRECTION_NTC_CON_TABLE_REQ:
			cmd_correction__GET_NTC_TBL(received);
			break;
		case OP_CORRECTION_NTC_CONSTANT_REQ:
			CmdCalibrationNTCConstantReq(received);
			break;
		}
		break;
	case CMD_SD_CARD:
		switch (received->option) {
		case OP_SDCARD_LIST:
			post_fs_job(FS_JOB_TYPE_QUERY_FILELIST);
			break;
		case OP_SDCARD_DOWNLOAD: {
			strncpy(request_filename, (const char *) received->data, 32);
			post_fs_job(FS_JOB_TYPE_DOWNLOAD_FILE);
			break;
		}
		case OP_SDCARD_DOWNLOAD_HEADER: {
			strncpy(request_filename, (const char *) received->data, 32);
			post_fs_job(FS_JOB_TYPE_DOWNLOAD_FILE_HEADER);
			break;
		}
		case OP_SDCARD_DELETE: {
			strncpy(request_filename, (const char *) received->data, 32);
			post_fs_job(FS_JOB_TYPE_DELETE_FILE);
			break;
		}
		case OP_SDCARD_FORMAT: {
			post_fs_job(FS_JOB_TYPE_FORMAT);
			break;
		}
		case OP_SDCARD_ERROR: {
			cmd_sd__GET_ERROR(received);
			break;
		}
		}
		break;
	case CMD_SLOT:
		switch (received->option) {
		case OP_SLOT_SET:
			break;
		case OP_SLOT_REQ:
			break;
		}
		break;
	case CMD_TIME:
		switch (received->option) {
		case OP_TIME_SET:
			cmd_time__SET_TIME(received);
			break;
		case OP_TIME_REQ:
			cmd_time__GET_TIME(received);
			break;
		}
		break;
	case CMD_SYS:
		switch (received->option) {
		case OP_FIRMWARE_VERSION_REQ:
			cmd_sys__GET_FW_VER(received);
			break;
		}
		break;

	default:
		break;
	}
}

void external_rx_task(void const * argument)
{
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_DMA(&huart1, ext_rx_buffer, ARRAY_LEN(ext_rx_buffer));

	ext_rx_q_id = osMessageCreate(osMessageQ(ext_rx_q), NULL);

	while (1) {
		osEvent e = osMessageGet(ext_rx_q_id, osWaitForever);
		if (e.status == osEventMessage)
			ext_rx_check();
	}
}

int send_to_external(uint8_t cmd, uint8_t option, void *data,
		uint16_t data_len, uint16_t data_padding_len, uint16_t buffer_len)
{
	static struct external_frame_tx ext_frm_tx = { 0 };

	memset(&ext_frm_tx, 0, sizeof(struct external_frame_tx));
	ext_frm_tx.cmd = cmd;
	ext_frm_tx.option = option;
	ext_frm_tx.len = buffer_len;
	if (data)
		memcpy(ext_frm_tx.data, data, data_len);
	ext_frm_tx.data_padding_len = data_padding_len;
	post_job(JOB_TYPE_TO_EXTERNAL, &ext_frm_tx, 17 + data_padding_len);

	return ext_frm_tx.len;
}

static void cmd_surveillance__SET_INTERVAL(struct external_frame_rx *msg)
{
	if (!msg) return;

	SysProperties.interval_ms = msg->interval_set.sec * 1000;

	send_to_external(CMD_TEMP_TEST, OP_TEMP_SAMPLE_RATE,
			&SysProperties.interval_ms, sizeof(uint32_t), 12, 32);
}

static void cmd_warning__SET_THRESHOLD(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->warning_set_threshold.slot_id;
	uint8_t channel = msg->warning_set_threshold.channel;
	float value = msg->warning_set_threshold.value;

	if (slot_id == 0xFF) {
		/* ctx.hard_job_processing = true; */
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__THRESHOLD_SET(s, channel, value);
			osDelay(1);
		}
	} else {
		struct slot_s *s = &ctx.slots[slot_id];
		request_to_internal__THRESHOLD_SET(s, channel, value);
	}
}

static void cmd_warning__GET_THRESHOLD(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__THRESHOLD_REQ(s);
			osDelay(1);
		}
	} else {
		struct slot_s *s = &ctx.slots[slot_id];
		request_to_internal__THRESHOLD_REQ(s);
	}
}

static void cmd_warning__SET_VARIATION(struct external_frame_rx *efx)
{
	if (!efx) return;

	uint8_t slot_id = efx->warning_set_variation.slot_id;
	float value = efx->warning_set_variation.value;

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			s->ntc.variation = value;
			request_to_internal__VARIATION_SET(s, value);
			osDelay(1);
		}
	} else {
		struct slot_s *s = &ctx.slots[slot_id];
		s->ntc.variation = value;
		request_to_internal__VARIATION_SET(s, value);
	}
}


static void cmd_warning__GET_VARIATION(struct external_frame_rx *efx)
{
	if (!efx) return;

	uint8_t slot_id = efx->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__VARIATION_REQ(s);
			osDelay(1);
		}
	} else {
		struct slot_s *s = &ctx.slots[slot_id];
		request_to_internal__VARIATION_REQ(s);
	}
}

static void cmd_compensated__SET(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->revision_apply_set.slot_id;
		uint8_t enabled = msg->revision_apply_set.enabled;

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__SET_COMPENSATED(s, enabled);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__SET_COMPENSATED(s, enabled);
			}
		}
	}
}

static void cmd_compensated__GET(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__GET_COMPENSATED(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__GET_COMPENSATED(s);
			}
		}
	}
}


static void cmd_compensated__SET_CONTACT_CONST(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				s->ntc.compensated.contact_const = *((float *) &msg->data[1]);
				request_to_internal__SET_COMPENSATED_CONTACT_CONST(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				s->ntc.compensated.contact_const = *((float *) &msg->data[1]);
				request_to_internal__SET_COMPENSATED_CONTACT_CONST(s);
			}
		}
	}
}

static void cmd_compensated__GET_CONTACT_CONST(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__GET_COMPENSATED_CONTACT_CONST(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__GET_COMPENSATED_CONTACT_CONST(s);
			}
		}
	}
}

static void cmd_compensated__SET_TR_CONST(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->revision_tr_const.slot_id;
		float r1 = msg->revision_tr_const.tr1;
		float r2 = msg->revision_tr_const.tr2;

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__SET_COMPENSATED_TR_CONST(s, r1, r2);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__SET_COMPENSATED_TR_CONST(s, r1, r2);
			}
		}
	}
}

static void cmd_compensated__GET_TR_CONST(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__GET_COMPENSATED_TR_CONST(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__GET_COMPENSATED_TR_CONST(s);
			}
		}
	}
}

static void cmd_correction__SET_RTD_CONST(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t count = 0;

	ctx.rtd.calibration_const = msg->rtd_calibration_const.v;

	float from_flash = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);

	if (ctx.rtd.calibration_const != from_flash) {
		do {
			doFlashWriteRevision(ctx.rtd.calibration_const);
			if (++count > 10)
				break;
			from_flash = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);
		} while (ctx.rtd.calibration_const != from_flash);
	}

	if (count > 10) {
		count = 0xFF;
		send_to_external(msg->cmd, msg->option, &count, 1, 12, 32);
		return;
	}

	send_to_external(msg->cmd, msg->option, &ctx.rtd.calibration_const, 4, 12, 32);
}

static void cmd_correction__GET_RTD_CONST(struct external_frame_rx *msg)
{
	if (!msg) return;
	send_to_external(msg->cmd, msg->option, &ctx.rtd.calibration_const, 4, 12, 32);
}


static void cmd_correction__CAL_NTC_TBL(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__CALIBRATION_NTC_TABLE_CAL(s);
			osDelay(1);
		}
	} else {
		if (slot_id < MAX_SLOT_NUM) {
			struct slot_s *s = &ctx.slots[slot_id];
			request_to_internal__CALIBRATION_NTC_TABLE_CAL(s);
		}
	}
}

static void cmd_correction__GET_NTC_TBL(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__CALIBRATION_NTC_TABLE_REQ(s);
			osDelay(1);
		}
	} else {
		if (slot_id < MAX_SLOT_NUM) {
			struct slot_s *s = &ctx.slots[slot_id];
			request_to_internal__CALIBRATION_NTC_TABLE_REQ(s);
		}
	}
}

static void CmdCalibrationNTCConstantSet(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__CALIBRATION_NTC_CONST_SET(s);
			osDelay(1);
		}
	} else {
		if (slot_id < MAX_SLOT_NUM) {
			struct slot_s *s = &ctx.slots[slot_id];
			request_to_internal__CALIBRATION_NTC_CONST_SET(s);
		}
	}
}


static void CmdCalibrationNTCConstantReq(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->data[0];

	if (slot_id == 0xFF) {
		FOREACH(struct slot_s *s, ctx.slots) {
			request_to_internal__CALIBRATION_NTC_CONST_REQ(s);
			osDelay(1);
		}
	} else {
		if (slot_id < MAX_SLOT_NUM) {
			struct slot_s *s = &ctx.slots[slot_id];
			request_to_internal__CALIBRATION_NTC_CONST_REQ(s);
		}
	}
}

static RTC_DateTypeDef setDate, getDate;
static RTC_TimeTypeDef setTime, getTime;

static void cmd_time__SET_TIME(struct external_frame_rx *msg)
{
	uint8_t res = TRUE;
	HAL_StatusTypeDef resHal;

	setDate.Year = msg->data[0];
	setDate.Month = msg->data[1];
	setDate.Date = msg->data[2];
	setDate.WeekDay = msg->data[6];
	setTime.Hours = msg->data[3];
	setTime.Minutes = msg->data[4];
	setTime.Seconds = msg->data[5];
	setTime.TimeFormat = RTC_HOURFORMAT_24;
	setTime.StoreOperation = RTC_STOREOPERATION_RESET;

	do {
		while (1) {
			resHal = HAL_RTC_SetDate(&hrtc, &setDate, RTC_FORMAT_BIN);
			if (resHal == HAL_OK)
				break;
			osDelay(1);
		}
		while (1) {
			resHal = HAL_RTC_SetTime(&hrtc, &setTime, RTC_FORMAT_BIN);
			if (resHal == HAL_OK)
				break;
			osDelay(1);
		}

		HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);

		if ((setDate.Year == getDate.Year) &&
			(setDate.Month == getDate.Month) &&
			(setDate.Date == getDate.Date) &&
			(setTime.Hours == getTime.Hours) &&
			(setTime.Minutes == getTime.Minutes) &&
			(setTime.Seconds == getTime.Seconds)) {
			break;
		}
	} while (1);

	SysTime.Date = getDate;
	SysTime.Time = getTime;
	SysProperties.time_synced = true;
	send_to_external(msg->cmd, msg->option, &res, 1, 12, 32);
}

static void cmd_time__GET_TIME(struct external_frame_rx *req)
{
	/* uint8_t res = 0; */
	/* send_to_external(CMD_TIME, OP_TIME_REQ, &res, 0, 12, 32); */
	send_to_external(req->cmd, req->option, NULL, 0, 12, 32);
}

static void cmd_sys__GET_FW_VER(struct external_frame_rx *req)
{
	send_to_external(req->cmd, req->option, (void *) firmware_version(), strlen(firmware_version()), 12, 32);
}

static void cmd_sd__GET_ERROR(struct external_frame_rx *efr)
{
	struct external_sd_get_error data = {
		.error = ctx.sd_last_error,
	};

	send_to_external(efr->cmd, efr->option, &data, 1, 12, 32);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *husart)
{
	DBG_LOG("%s\r\n", __func__);
}
