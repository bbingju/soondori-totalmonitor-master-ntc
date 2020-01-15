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

extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t ReadFileBuf[MAX_485_BUF_LEN];
uint8_t FindFilelistFlag = 0; //0 : 파일 리스트 검색 안하는중 / 1 : 파일 리스트 검색중
int ext_tx_completed = 1;

static void parse_rx(const void *data, size_t len);

static void CmdWarningTempSet(struct external_frame_rx *);
static void CmdWarningTempReq(struct external_frame_rx *);
static void CmdRevisionApplySet(struct external_frame_rx *);
static void CmdRevisionApplyReq(struct external_frame_rx *);
static void CmdRevisionConstantSet(struct external_frame_rx *);
static void CmdRevisionConstantReq(struct external_frame_rx *);
static void CmdRevisionTRConstantSet(struct external_frame_rx *);
static void CmdRevisionTRConstantReq(struct external_frame_rx *);
static void CmdCalibrationRTDConstSet(struct external_frame_rx *);
static void CmdCalibrationRTDConstReq(struct external_frame_rx *);
static void CmdCalibrationNTCTableCal(struct external_frame_rx *);
static void CmdCalibrationNTCTableReq(struct external_frame_rx *);
static void CmdCalibrationNTCConstantSet(struct external_frame_rx *);
static void CmdCalibrationNTCConstantReq(struct external_frame_rx *);
static void doSetTime(struct external_frame_rx *);
static void doGetTime(struct external_frame_rx *);
static void handle_firmware_version_req(struct external_frame_rx *);

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
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_FLAG_FEIF1_5);
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

void receive_from_external(struct external_frame_rx *received)
{
	/* DBG_LOG("ext rx [%s::%s]: ", ext_cmd_str(received->cmd), */
	/* 	ext_option_str(received->cmd, received->option)); */
	/* DBG_DUMP(received->data, 22); */

	switch (received->cmd) {
	case CMD_TEMP_TEST:
		switch(received->option) {
		case OP_TEMP_START_RX:
			SysProperties.start_flag = TRUE;
			break;
		case OP_TEMP_STOP:
			SysProperties.start_flag = FALSE;
			break;
		case OP_TEMP_SAMPLE_RATE:
			doSaveIntervalTime(received);
			break;
		}
		break;
	case CMD_WARNING_TEMP:
		switch (received->option) {
		case OP_WARNING_TEMP_SET:
			CmdWarningTempSet(received);
			break;
		case OP_WARNING_TEMP_REQ:
			CmdWarningTempReq(received);
			break;
		}
		break;
	case CMD_REVISION:
		switch (received->option) {
		case OP_REVISION_APPLY_SET:
			CmdRevisionApplySet(received);
			break;
		case OP_REVISION_APPLY_REQ:
			CmdRevisionApplyReq(received);
			break;
		case OP_REVISION_CONSTANT_SET:
			CmdRevisionConstantSet(received);
			break;
		case OP_REVISION_CONSTANT_REQ:
			CmdRevisionConstantReq(received);
			break;
		case OP_REVISION_TR_CONST_SET:
			CmdRevisionTRConstantSet(received);
			break;
		case OP_REVISION_TR_CONST_REQ:
			CmdRevisionTRConstantReq(received);
			break;
		}
		break;
	case CMD_CALIBRATION:
		switch (received->option) {
		case OP_CALIBRATION_RTD_CONSTANT_SET:
			CmdCalibrationRTDConstSet(received);
			break;
		case OP_CALIBRATION_NTC_CON_TABLE_CAL:
			CmdCalibrationNTCTableCal(received);
			break;
		case OP_CALIBRATION_NTC_CONSTANT_SET:
			CmdCalibrationNTCConstantSet(received);
			break;
		case OP_CALIBRATION_RTD_CONSTANT_REQ:
			CmdCalibrationRTDConstReq(received);
			break;
		case OP_CALIBRATION_NTC_CON_TABLE_REQ:
			CmdCalibrationNTCTableReq(received);
			break;
		case OP_CALIBRATION_NTC_CONSTANT_REQ:
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
			download_is_header_request = false;
			strncpy(requested_path, received->data, 64);
			post_fs_job(FS_JOB_TYPE_DOWNLOAD_FILE);
			break;
		}
		case OP_SDCARD_DOWNLOAD_HEADER: {
			download_is_header_request = true;
			strncpy(requested_path, received->data, 64);
			post_fs_job(FS_JOB_TYPE_DOWNLOAD_FILE);
			break;
		}
		case OP_SDCARD_DELETE:
			break;
		case OP_SDCARD_FORMAT:
			break;
		case OP_SDCARD_ERROR:
			break;
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
			doSetTime(received);
			break;
		case OP_TIME_REQ:
			doGetTime(received);
			break;
		}
		break;
	case CMD_FW:
		switch (received->option) {
		case OP_FIRMWARE_VERSION_REQ:
			handle_firmware_version_req(received);
			break;
		}
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
	post_job(JOB_TYPE_TO_EXTERNAL, &ext_frm_tx, sizeof(struct external_frame_tx)/* ext_frm_tx.len - 20 */);

	return ext_frm_tx.len;
}

void DoSendFile(struct external_frame_rx *msg)
{
	DoSendFileOpen(msg);
	DoSendFileBodyPacket(0x0000, (UINT)(16 * 1024));
	DoSendFileClose();
}

void DoSendFileOpen(struct external_frame_rx *msg)
{
#if 0
	uint8_t t[1] = {0};
	FRESULT res = FR_OK;
	uint8_t fileNameLen = 0;
	FILINFO fno;

	memset(sdValue.sendFileName, 0x00, sizeof(sdValue.sendFileName));
	sdValue.sendFileName[0] = '0';
	sdValue.sendFileName[1] = ':';

	for (sdValue.sendFileNameLen = 0; sdValue.sendFileNameLen < 22;
	     sdValue.sendFileNameLen++) {
		if (msg->data[/* Rx485Data[7 +  */ sdValue.sendFileNameLen] == '>')
			break;

		sdValue.sendFileName[13 + sdValue.sendFileNameLen] =
			msg->data[/* Rx485Data[7 + */ sdValue.sendFileNameLen];
	}
	sdValue.sendFileName[2] = '/';
	sdValue.sendFileName[3] = '2';
	sdValue.sendFileName[4] = '0';
	sdValue.sendFileName[5] = sdValue.sendFileName[14];
	sdValue.sendFileName[6] = sdValue.sendFileName[15];
	sdValue.sendFileName[7] = '/';
	sdValue.sendFileName[8] = sdValue.sendFileName[16];
	sdValue.sendFileName[9] = sdValue.sendFileName[17];
	sdValue.sendFileName[10] = '/';
	sdValue.sendFileName[11] = sdValue.sendFileName[18];
	sdValue.sendFileName[12] = sdValue.sendFileName[19];

	res = f_stat((const TCHAR *)sdValue.sendFileName, &fno);

	if (res == FR_OK) {
		res = f_open(&sdValue.sendFileObject, sdValue.sendFileName,
			FA_OPEN_EXISTING | FA_READ);
		if (res == FR_OK) { //파일 정상 오픈
			sdValue.sdState = SCS_OK;
			send_to_external(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_HEADER,
					sdValue.sendFileName, fileNameLen, 36, 56);
		} else {	//파일 오픈 에러
			sdValue.sdState = SCS_OPEN_ERROR;
			t[0] = SCS_OPEN_ERROR;
			send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
		}
	}
#endif	/* 0 */
}

void DoSendFileBodyPacket(uint32_t Offset, UINT packetSize)
{
#if 0
	/* uint8_t t[1] = {0}; */
	int32_t ReadSize;
	uint16_t i = 0;
	uni4Byte temp;

	/* if(osSemaphoreGetCount(CountingSem485TxHandle) == 0) */
	/*     osSemaphoreRelease(CountingSem485TxHandle); */

	do {
		memset(ReadFileBuf, 0x00, sizeof(ReadFileBuf));
		temp.UI32 = Offset + (packetSize * i);
		ReadFileBuf[0] = temp.UI8[0];
		ReadFileBuf[1] = temp.UI8[1];
		ReadFileBuf[2] = temp.UI8[2];
		ReadFileBuf[3] = temp.UI8[3];
		// util_mem_cpy(&ReadFileBuf[0], &temp.UI8[0], 4);
		temp.UI32 = Offset + (packetSize * (i + 1));
		ReadFileBuf[4] = temp.UI8[0];
		ReadFileBuf[5] = temp.UI8[1];
		ReadFileBuf[6] = temp.UI8[2];
		ReadFileBuf[7] = temp.UI8[3];
		// util_mem_cpy(&ReadFileBuf[4], &temp.UI8[0], 4);

		ReadSize = DoSendFileRead(Offset + (packetSize * i), packetSize);
		send_to_external(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_BODY,
				&ReadFileBuf[0], ReadSize + 8, packetSize + 8,
				packetSize + 8 + 20);
		/* doMakeSend485DataDownLoad(tx485DataDMA, CMD_SD_CARD,
		 * OP_SDCARD_DOWNLOAD_BADY, &ReadFileBuf[0], ReadSize + 8, packetSize +
		 * 8, packetSize + 8 + 20); */
		/* SendUart485String(&tx485DataDMA[0], packetSize + 8 + 20); */

		/*		uint16_t len = (packetSize + 8 + 20);
				uint16_t ct  = (len / 32);

				for(int j = 0; j < ct; j++)
				{
				SendUart485String(&tx485DataDMA[j * 32], 32);
				len -= 32;
				}
				osDelay(1);
				SendUart485String(&tx485DataDMA[j * 32], len);
		*/
		if (packetSize != ReadSize) //마지막 페킷
		{
			osDelay(1);
			break;
		}
		i++;
	} while (1);

	osDelay(1);
#endif //0
}

void DoSendFileClose(void)
{
#if 0
	FRESULT res = FR_OK;
	uint8_t t[1] = {0};

	res = f_close(&sdValue.sendFileObject);
	if (res == FR_OK) //파일 정상으로 닫힘
	{
		sdValue.sdState = SCS_OK;
		send_to_external(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_FOOTER,
				sdValue.sendFileName, sdValue.sendFileNameLen,
				36, 56);
	} else //파일 닫기 에러
	{
		sdValue.sdState = SCS_CLOSE_ERROR;
		t[0] = SCS_CLOSE_ERROR;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
	}
#endif /* 0 */
}

int32_t DoSendFileRead(FSIZE_t Offset, UINT ReadSize)
{
#if 0
	FRESULT res = FR_OK;
	UINT br = 0;
	uint8_t t[1] = {0};

	res = f_lseek(&sdValue.sendFileObject, Offset);
	if (res != FR_OK) {
		sdValue.sdState = SCS_OK;
		t[0] = SCS_SEEK_ERROR;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
	}

	res = f_read(&sdValue.sendFileObject, &ReadFileBuf[8], ReadSize, &br);
	if (res != FR_OK) {
		sdValue.sdState = SCS_READ_ERROR;
		t[0] = SCS_READ_ERROR;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
	}

	return (int32_t)br;
#endif
	return 0;
}

/* void DoReadFileList(struct external_frame_rx *msg) */
/* { */
/* 	uint8_t t[1] = {0}; */
/* 	TCHAR root_directory[3] = "0:"; */

/* 	send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_START, t, 0, 12, 32); */

/* 	memset(&sdValue.scanDir[0], 0x00, sizeof(sdValue.scanDir)); */
/* 	sdValue.scanDirDeep = 0; */
/* 	memset(&sdValue.scanFilePath, 0x00, sizeof(sdValue.scanFilePath)); */

/* 	if (msg->data[0] /\* Rx485Data[7] *\/ == '.') //루트를 요청 했을 경우 */
/* 	{ */
/* 		osDelay(1); */
/* 		scan_files(root_directory); */
/* 	} else {			//경로 지정 했을 경우 */
/* 		memset(&sdValue.scanReadFileName[0], 0x00, sizeof(sdValue.scanReadFileName)); */
/* 		sdValue.scanReadFileName[0] = '0'; */
/* 		sdValue.scanReadFileName[1] = ':'; */

/* 		for (int i = 0; i < 22; i++) { */
/* 			if (msg->data[/\* Rx485Data[7 + *\/ i] == '>') */
/* 				break; */

/* 			sdValue.scanReadFileName[2 + i] = msg->data[/\* Rx485Data[7 + *\/ i]; */
/* 		} */

/* 		while (scan_files((char *)sdValue.scanReadFileName) != FR_OK) { */
/* 			osDelay(1); */
/* 		} */
/* 	} */

/* 	send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_END, t, 0, 12, 32); */
/* } */

void doSaveIntervalTime(struct external_frame_rx *msg) //샘플레이트
{
	if (!msg) return;

	SysProperties.interval_ms = msg->interval_set.sec * 1000;

	send_to_external(CMD_TEMP_TEST, OP_TEMP_SAMPLE_RATE,
			&SysProperties.interval_ms, sizeof(uint32_t), 12, 32);
}

static void CmdWarningTempSet(struct external_frame_rx *msg)
{
	if (!msg) return;

	uint8_t slot_id = msg->warning_temp_set.slot_id;
	uint8_t channel = msg->warning_temp_set.channel;
	float value = msg->warning_temp_set.value;

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

static void CmdWarningTempReq(struct external_frame_rx *msg)
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

static void CmdRevisionApplySet(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->revision_apply_set.slot_id;
		uint8_t enabled = msg->revision_apply_set.enabled;

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__REVISION_APPLY_SET(s, enabled);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__REVISION_APPLY_SET(s, enabled);
			}
		}
	}
}

static void CmdRevisionApplyReq(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__REVISION_APPLY_REQ(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__REVISION_APPLY_REQ(s);
			}
		}
	}
}


static void CmdRevisionConstantSet(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				s->ntc.revision_const = *((float *) &msg->data[1]);
				request_to_internal__REVISION_CONST_SET(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				s->ntc.revision_const = *((float *) &msg->data[1]);
				request_to_internal__REVISION_CONST_SET(s);
			}
		}
	}
}

static void CmdRevisionConstantReq(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__REVISION_CONST_REQ(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__REVISION_CONST_REQ(s);
			}
		}
	}
}

static void CmdRevisionTRConstantSet(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->revision_tr_const.slot_id;
		float r1 = msg->revision_tr_const.tr1;
		float r2 = msg->revision_tr_const.tr2;

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__REVISION_TR_CONST_SET(s, r1, r2);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__REVISION_TR_CONST_SET(s, r1, r2);
			}
		}
	}
}

static void CmdRevisionTRConstantReq(struct external_frame_rx *msg)
{
	if (msg) {
		uint8_t slot_id = msg->data[0];

		if (slot_id == 0xFF) {
			FOREACH(struct slot_s *s, ctx.slots) {
				request_to_internal__REVISION_TR_CONST_REQ(s);
				osDelay(1);
			}
		} else {
			if (slot_id < MAX_SLOT_NUM) {
				struct slot_s *s = &ctx.slots[slot_id];
				request_to_internal__REVISION_TR_CONST_REQ(s);
			}
		}
	}
}

static void CmdCalibrationRTDConstSet(struct external_frame_rx *msg)
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

static void CmdCalibrationRTDConstReq(struct external_frame_rx *msg)
{
	if (!msg) return;
	send_to_external(msg->cmd, msg->option, &ctx.rtd.calibration_const, 4, 12, 32);
}


static void CmdCalibrationNTCTableCal(struct external_frame_rx *msg)
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

static void CmdCalibrationNTCTableReq(struct external_frame_rx *msg)
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

static void doSetTime(struct external_frame_rx *msg)
{
	uint8_t res = TRUE;
	HAL_StatusTypeDef resHal;
	RTC_DateTypeDef setDate, getDate;
	RTC_TimeTypeDef setTime, getTime;

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
			resHal = HAL_RTC_SetTime(&hrtc, &setTime, RTC_FORMAT_BIN);
			if (resHal == HAL_OK)
				break;
			osDelay(1);
		}
		while (1) {
			resHal = HAL_RTC_SetDate(&hrtc, &setDate, RTC_FORMAT_BIN);
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

	SysProperties.time_synced = true;
	send_to_external(msg->cmd, msg->option, &res, 1, 12, 32);
}

static void doGetTime(struct external_frame_rx *req)
{
	/* uint8_t res = 0; */
	/* send_to_external(CMD_TIME, OP_TIME_REQ, &res, 0, 12, 32); */
	send_to_external(req->cmd, req->option, NULL, 0, 12, 32);
}

static void handle_firmware_version_req(struct external_frame_rx *req)
{
	send_to_external(req->cmd, req->option, (void *) firmware_version(), strlen(firmware_version()), 12, 32);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *husart)
{
	DBG_LOG("%s\r\n", __func__);
}
