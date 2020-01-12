#include "fs_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "app_ctx.h"
#include "frame.h"
#include "external_uart_task.h"
#include "debug.h"

#include <string.h>

static osMessageQDef(fs_job_q, 16, FS_JOB_TYPE_E);
static osMessageQId (fs_job_q_id);

static FIL logfp;
static FIL metafp;
static uint8_t fs_buffer[4096];

static void handle_fs_job(FS_JOB_TYPE_E type);
static void save_log();
static void answer_log_filelist();
static int start_logfile_listing(FIL *);
static void end_logfile_listing(FIL *);
static char *current_log_dirname();
static char *current_log_filename();
static char *current_log_fullpath();
static FRESULT check_n_create_dir(const char *dirname);
static FRESULT check_dir_for_current_date(void);
static void write_log_file_header(FIL *fil);
static FRESULT write_data(FIL *fil);
static FRESULT update_metafile(FIL *fil);

extern app_ctx_t ctx;
extern uint8_t wData[];

__PACKED_STRUCT ymd {
	uint8_t y;
	uint8_t m;
	uint8_t d;
};

static int translate_ymd(struct ymd *pymd, const char *str_ymd)
{
	char *n;

	if (!pymd || !str_ymd)
		return -1;

	n = strndup(str_ymd, 2);
	pymd->y = (uint8_t)strtol(n, NULL, 10);
	free(n);

	/* month */
	n = strndup(str_ymd + 2, 2);
	pymd->m = (uint8_t)strtol(n, NULL, 10);
	free(n);

	/* day */
	n = strndup(str_ymd + 4, 2);
	pymd->d = (uint8_t)strtol(n, NULL, 10);
	free(n);

	return 0;
}

int post_fs_job(FS_JOB_TYPE_E type)
{
	if (!ctx.sd_inserted)
		return -1;

	if (osMessagePut(fs_job_q_id, type, 0) != osOK)
		return -1;
	return 0;
}

void fs_task(void const *arg)
{
	fs_job_q_id = osMessageCreate(osMessageQ(fs_job_q), NULL);

	while (1) {

		osEvent e = osMessageGet(fs_job_q_id, osWaitForever);
		if (e.status == osEventMessage) {
			handle_fs_job(e.value.v);
		}
	}
}

static void handle_fs_job(FS_JOB_TYPE_E type)
{
	switch (type) {
	case FS_JOB_TYPE_SAVE_LOG:
		save_log();
		break;
	case FS_JOB_TYPE_QUERY_FILELIST:
		answer_log_filelist();
		break;
	case FS_JOB_TYPE_DOWNLOAD_FILE:
		break;
	default:
		return;
	}
}

static void save_log()
{
	FRESULT ret = FR_OK;
	bool new_file_created = false;

	/* bool logfile_existed = false; */

	/* DBG_LOG("%s %u\n", __func__, osKernelSysTick()); */

	/* DBG_LOG("%s\n", current_log_filename()); */

	/* check the directory for current date */
	ret = check_dir_for_current_date();

	/* check whether the current log file exist */

	/* open log file */
	char *fullpath = current_log_fullpath();
	ret = f_open(&logfp, fullpath, FA_OPEN_APPEND | FA_WRITE);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): f_open during %s\n", ret, __func__);
		return;
	}

	if (f_size(&logfp) == 0) {
		new_file_created = true;
		write_log_file_header(&logfp);
	}
	ret = write_data(&logfp);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): write_data during %s\n", ret, __func__);
		f_close(&logfp);
		return;
	}

	f_sync(&logfp);
	f_close(&logfp);

	if (new_file_created) {
		update_metafile(&metafp);
	}
}

static void answer_log_filelist()
{
	FRESULT ret = FR_OK;
	static uint8_t _time_data[48] = { 0 };

	ret = f_open(&metafp, "0://.metafile", FA_OPEN_EXISTING | FA_READ);
	if (ret != FR_OK) {
		goto ret_error;
	}

	int dates = start_logfile_listing(&metafp);

	char buffer[64] = { 0 };
	uint16_t sequence_index = 0;
	uint8_t time_index = 0;
	/* char *n = NULL; */
	struct ymd ymd = { 0 };

	f_lseek(&metafp, 0);

	while (f_gets(buffer, 64, &metafp)) {

		if (buffer[0] != '\t') {

			if (sequence_index > 0) {
				send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_BODY, _time_data,
						5 + time_index, 5 + time_index, 20 + 5 + time_index);
			}

			time_index = 0;
			*((uint16_t *)&_time_data[0]) = sequence_index++;

			translate_ymd(&ymd, buffer);
			_time_data[2] = ymd.y;
			_time_data[3] = ymd.m;
			_time_data[4] = ymd.d;

			DBG_LOG("header: ");
			DBG_DUMP(_time_data, 5);
		} else {
			char *n;

			/* hour */
			n = strndup(&buffer[8], 2);
			_time_data[5 + time_index++] = (uint8_t)strtol(n, NULL, 10);
			free(n);
		}
		/* DBG_LOG("%s", buffer); */
	}

	/* last body */
	if (sequence_index == dates)
		send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_BODY, _time_data,
				5 + time_index, 5 + time_index, 20 + 5 + time_index);


	end_logfile_listing(&metafp);

ret_error:
	f_close(&metafp);
	/* return ret; */
}

static int start_logfile_listing(FIL *fil)
{
	int date_count = 0;
	char buffer[64] = { 0 };

	f_lseek(fil, 0);

	while (f_gets(buffer, 64, fil)) {

		if (buffer[0] != '\t') {
			char *n = NULL;

			/* year */
			n = strndup(&buffer[0], 2);
			fs_buffer[date_count * 4 + 0] = (uint8_t)strtol(n, NULL, 10);
			free(n);

			/* month */
			n = strndup(&buffer[2], 2);
			fs_buffer[date_count * 4 + 1] = (uint8_t)strtol(n, NULL, 10);
			free(n);

			/* day */
			n = strndup(&buffer[4], 2);
			fs_buffer[date_count * 4 + 2] = (uint8_t)strtol(n, NULL, 10);
			free(n);

			fs_buffer[date_count * 4 + 3] = 0x10;

			date_count++;
			DBG_LOG("header (%d): ", date_count);
			for (int i = 0; i < date_count; i++)
				DBG_DUMP(&fs_buffer[i * 4], 4);
		}
	}

	if (date_count > 0)
		send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_START, fs_buffer,
				4 * date_count, 4 * date_count, 20 + 4 * date_count);

	return date_count;
}

static void end_logfile_listing(FIL *fil)
{
	send_to_external(CMD_SD_CARD, OP_SDCARD_LIST_END, NULL, 0, 0, 20);
}

static char *current_log_dirname()
{
	static char _dirname[24] = { 0 };
	RTC_DateTypeDef *d = &SysTime.Date;

	sprintf(_dirname, "0://20%02d/%02d/%02d",
		d->Year, d->Month, d->Date);

	return _dirname;
}

static char *current_log_filename()
{
	static char _filename[24] = { 0 };
	RTC_DateTypeDef *d = &SysTime.Date;

	sprintf(_filename, "%02d%02d%02d_%02d%02d%02d.ske",
		d->Year, d->Month, d->Date,
		SysTime.Time.Hours, 0, 0);

	return _filename;
}

static char *current_log_fullpath()
{
	static char _fullpath[48] = { 0 };
	RTC_DateTypeDef *d = &SysTime.Date;
	RTC_TimeTypeDef *t = &SysTime.Time;

	sprintf(_fullpath, "0://20%02d/%02d/%02d/%02d%02d%02d_%02d%02d%02d.ske",
		d->Year, d->Month, d->Date,
		d->Year, d->Month, d->Date,
		t->Hours, 0, 0);

	return _fullpath;
}

static FRESULT check_n_create_dir(const char *dirname)
{
	FRESULT result = FR_OK;
	DIR dir;

	result = f_opendir(&dir, dirname);
	if (result == FR_OK) {
		f_closedir(&dir);
		return result;
	}

	return f_mkdir(dirname);
}

static FRESULT check_dir_for_current_date(void)
{
	TCHAR dirname[16] = {0};
	FRESULT res = FR_OK;
	RTC_DateTypeDef *d = &SysTime.Date;

	sprintf(dirname, "20%02d", d->Year);
	res = check_n_create_dir(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

	sprintf(&dirname[strlen(dirname)], "/%02d", d->Month);
	res = check_n_create_dir(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

	sprintf(&dirname[strlen(dirname)], "/%02d", d->Date);
	res = check_n_create_dir(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

ret:
	return res;
}

static void write_log_file_header(FIL *fil)
{
	FRESULT res;
	uint16_t i;
	uni4Byte timezone;

	res = f_lseek(fil, 0x0000); // File 확장
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}

	for (i = 0; i < FILE_HEADER_SIZE; i++) {
		f_putc(0x00, fil); // File Header 초기화
	}

	res = f_lseek(fil, FILE_ADD_FILE_CONFIRMATION); // File Confirmation
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_printf(fil, (const TCHAR *)"This is a TempMon File. devteam.");

	res = f_lseek(fil, FILE_ADD_MODEL_NUMBER); // Model No.
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_printf(fil, "TTM-128");

	res = f_lseek(fil, FILE_ADD_HW_VERSION); // H/W version
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_printf(fil, "1.10");

	res = f_lseek(fil, FILE_ADD_FW_VERSION); // F/W version
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_printf(fil, "1.00");

	res = f_lseek(fil,
		FILE_ADD_TEST_DATE_TIME); // Start Date Time
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_printf(fil, "20%02d%02d%02d%02d%02d%02d",
		SysTime.Date.Year, // file
		SysTime.Date.Month, SysTime.Date.Date,
		SysTime.Time.Hours, SysTime.Time.Minutes,
		SysTime.Time.Seconds);
	res = f_lseek(fil, FILE_ADD_TIME_ZONE); // Time Zone
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	timezone.Float = 9;
	f_putc(timezone.UI8[0], fil);
	f_putc(timezone.UI8[1], fil);
	f_putc(timezone.UI8[2], fil);
	f_putc(timezone.UI8[3], fil);

	res = f_lseek(fil, FILE_ADD_MCU_UUID); // MCU UUID
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	f_putc(SysProperties.mcuUUID[0].UI8[0], fil);
	f_putc(SysProperties.mcuUUID[0].UI8[1], fil);
	f_putc(SysProperties.mcuUUID[0].UI8[2], fil);
	f_putc(SysProperties.mcuUUID[0].UI8[3], fil);
	f_putc(SysProperties.mcuUUID[1].UI8[0], fil);
	f_putc(SysProperties.mcuUUID[1].UI8[1], fil);
	f_putc(SysProperties.mcuUUID[1].UI8[2], fil);
	f_putc(SysProperties.mcuUUID[1].UI8[3], fil);
	f_putc(SysProperties.mcuUUID[2].UI8[0], fil);
	f_putc(SysProperties.mcuUUID[2].UI8[1], fil);
	f_putc(SysProperties.mcuUUID[2].UI8[2], fil);
	f_putc(SysProperties.mcuUUID[2].UI8[3], fil);

	res = f_lseek(fil, FILE_ADD_SLOT_USE); // slot use
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	for (int i = 0; i < 4; i++)
		f_putc(ctx.slots[i].inserted, fil);

	res = f_lseek(fil, FILE_ADD_SLOT_TYPE); // slot type
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}
	for (int i = 0; i < 4; i++)
		f_putc(ctx.slots[i].type, fil);

	res = f_lseek(fil, FILE_ADD_TEST_DATA); // file write position
	if (res != FR_OK) {
		/* sdValue.sdState = SCS_SEEK_ERROR; */
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return;
	} else {
		/* sdValue.sdState = SCS_OK; */
	}

	/* res = f_sync(fil); */
	/* if (res != FR_OK) { */
	/* 	/\* sdValue.sdState = SCS_SYNC_ERROR; *\/ */
	/* 	/\* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, *\/ */
	/* 	/\*                        (uint8_t *)sdValue.sdState, 1, 12, 32); *\/ */
	/* 	return; */
	/* } else { */
	/* 	/\* sdValue.sdState = SCS_OK; *\/ */
	/* } */
}

static FRESULT write_data(FIL *fil)
{
	uint8_t sensorCount = 0;
	uint16_t writeLen = 0;
	UINT retCount;
	FRESULT res = FR_OK;
	// FSIZE_t size;

	//메모리 초기화
	memset(wData, 0x00, 420);

	//시간
	wData[0] = SysTime.Date.Year;
	wData[1] = SysTime.Date.Month;
	wData[2] = SysTime.Date.Date;
	wData[3] = SysTime.Time.Hours;
	wData[4] = SysTime.Time.Minutes;
	wData[5] = SysTime.Time.Seconds;
	wData[6] = (uint8_t)((SysTime.Time.SubSeconds & 0x0000FF00) >> 8);
	wData[7] = (uint8_t)(SysTime.Time.SubSeconds & 0x000000FF);

	//채널
	FOREACH(struct slot_s *s, ctx.slots) {
		for (int i = 0; i < 16; i++) {
			if (s->ntc.channel_states[i] != CHANNEL_STATE_DISCONNECTED) {
				wData[34 + (sensorCount * 6)] = s->id * 16 + i; // 채널 번호
				wData[35 + (sensorCount * 6)] = s->ntc.channel_states[i]; // 센서 상태
				*((float *)&wData[36 + (sensorCount * 6)]) = s->ntc.temperatures[i]; // 센서값 저장
				sensorCount++; // 사용 채널수 확인
			}
		}
	}
	/* for (int j = 0; j < 4; j++) { */
	/*     for (int i = 0; i < 16; i++) { */
	/*         if (TestData.sensorState[j][i] != LDM_DONOT_CONNECT) { */
	/*             wData[34 + (sensorCount * 6)] = j * 16 + i; // 채널 번호 */
	/*             wData[35 + (sensorCount * 6)] = */
	/*                 TestData.sensorState[j][i]; // 센서 상태 */
	/*             memcpy(&wData[36 + (sensorCount * 6)], */
	/*                    &TestData.temperatures[j][i], 4); // 센서값 저장 */
	/*             sensorCount++; // 사용 채널수 확인 */
	/*         } */
	/*     } */
	/* } */

	if (sensorCount == 0) //센서가 모두 연결 안되있을때는 통과
	{
		return res;
	}

	wData[8] = sensorCount;

	//보드 센서 복사
	float v;
	for (int i = 0; i < 4; i++) {
		switch (i) {
		case 0:
			v = ctx.battery;
			break;
		case 1:
			v = ctx.rtd.temperature;
			break;
		case 2:
			v = ctx.temperature;
			break;
		case 3:
			v = ctx.humidity;
			break;
		}

		wData[10 + (i * 6)] = MCU_BOARD_BATTERY + i; //채널 번호, 0xEB ~ 0xEE
		wData[11 + (i * 6)] = 0x00; //보드 센서의 상태값은 없음.
		*((float *)&wData[12 + (i * 6)]) = v;
	}

	res = f_lseek(fil, f_size(fil));
	if (res != FR_OK) {
		sdValue.sdState = SCS_SEEK_ERROR;
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return res;
	} else {
		sdValue.sdState = SCS_OK;
	}

	writeLen = (sensorCount * 6) + 34;
	res = f_write(fil, &wData[0], writeLen, &retCount);
	if (res != FR_OK) {
		sdValue.sdState = SCS_WRITE_ERROR;
		/* send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
		/*                        (uint8_t *)sdValue.sdState, 1, 12, 32); */
		return res;
	} else {
		sdValue.sdState = SCS_OK;
	}

	/* res = f_sync(fil); */
	/* if (res != FR_OK) { */
	/*     sdValue.sdState = SCS_SYNC_ERROR; */
	/*     send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, */
	/*                            (uint8_t *)sdValue.sdState, 1, 12, 32); */
	/*     return; */
	/* } else { */
	/*     sdValue.sdState = SCS_OK; */
	/* } */
	return res;
}

static FRESULT update_metafile(FIL *fil)
{
	/* char buffer[64] = { 0 }; */
	FRESULT ret = FR_OK;

	ret = f_open(fil, "0://.metafile", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): f_open metafile during %s\n", ret, __func__);
		return ret;
	}

	/* /\* move pointer to last line & read the line *\/ */
	/* f_lseek(fil, f_size(fil) - 20); */
	/* f_gets(buffer, 19, fil); */


	/* if (buffer[0] != '\t') { */
	/* 	DBG_LOG("error: can not find last line of metafile\n"); */
	/* 	f_close(fil); */
	/* 	return 100; */
	/* } */

	/* struct ymd last_ymd = { 0 }; */
	/* translate_ymd(&last_ymd, &buffer[1]); */
	/* if (last_ymd.d != SysTime.Date.Date) { */

	/* } */

	f_lseek(fil, f_size(fil));
	f_printf(fil, "\t%s\n", current_log_filename());

	f_sync(fil);
	f_close(fil);

	return ret;
}
