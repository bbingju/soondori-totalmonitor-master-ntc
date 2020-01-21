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

#define FILE_CHUNCK_SIZE 4096
#define FILE_HEADER_SIZE 4096

/* static FIL logfp; */
static FIL downloadfp;
static uint8_t fs_buffer[FILE_CHUNCK_SIZE + 4 + 20];
static uint8_t fs_data_buffer[FILE_CHUNCK_SIZE + 4];
char request_filename[32];
static char requested_path[128];
bool download_is_header_request = false;

static void handle_fs_job(FS_JOB_TYPE_E type);
static void save_log();
static void send_log_filelist();
static int send_log_file(const char *filename, bool is_header);
static int delete_log_file(const char *filename);

static int start_logfile_listing(FIL *);
static void end_logfile_listing(FIL *);
static char *current_log_dirname();
static char *current_log_filename();
static char *current_log_fullpath();
static FRESULT check_n_create_dir(const char *dirname);
static FRESULT check_dir_for_current_date(void);
static FRESULT write_log_file_header(FIL *fil);
static FRESULT write_data(FIL *fil);
static FRESULT update_metafile(FIL *fil);
static void send_file_header(FIL *fil);
static void send_file_overall(FIL *fil);
static void _send_fs_packet(uint8_t cmd, uint8_t option, void *data, size_t datasize, size_t arraysize);

extern app_ctx_t ctx;
extern int ext_tx_completed;

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
	case FS_JOB_TYPE_SAVE_LOG: {
		uint32_t elapsed = osKernelSysTick();
		save_log();
		DBG_LOG("Saving a log elapsed %u\n", osKernelSysTick() - elapsed);
		break;
	}
	case FS_JOB_TYPE_QUERY_FILELIST:
		send_log_filelist();
		break;
	case FS_JOB_TYPE_DOWNLOAD_FILE_HEADER:
	case FS_JOB_TYPE_DOWNLOAD_FILE: {
		uint32_t elapsed = osKernelSysTick();
		struct ymd ymd;
		/* strcpy(request_filename, "200119_100000.ske"); /\* for test *\/ */
		translate_ymd(&ymd, request_filename);
		sprintf(requested_path, "0://20%02d/%02d/%02d/%s",
			ymd.y, ymd.m, ymd.d, request_filename);
		bool is_header = (type == FS_JOB_TYPE_DOWNLOAD_FILE) ? false : true;
		if (send_log_file(requested_path, is_header) != 0) {
			send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
		}
		DBG_LOG("Download file elapsed %u\n", osKernelSysTick() - elapsed);
		break;
	}
	case FS_JOB_TYPE_DELETE_FILE: {
		struct ymd ymd;
		translate_ymd(&ymd, request_filename);
		sprintf(requested_path, "0://20%02d/%02d/%02d/%s",
			ymd.y, ymd.m, ymd.d, request_filename);
		if (delete_log_file(requested_path) != 0) {
			send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
		}
		break;
	}
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
	ret = f_open(&ctx.logfd, fullpath, FA_OPEN_APPEND | FA_WRITE);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): f_open during %s\n", ret, __func__);
		ctx.sd_last_error = SD_RET_OPEN_ERR;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
		return;
	}

	if (f_size(&ctx.logfd) == 0) {
		new_file_created = true;
		ret = write_log_file_header(&ctx.logfd);
		if (ret != FR_OK) {
			send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
			f_close(&ctx.logfd);
			return;
		}
	}

	ret = write_data(&ctx.logfd);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): write_data during %s\n", ret, __func__);
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
		f_close(&ctx.logfd);
		return;
	}

	f_sync(&ctx.logfd);
	f_close(&ctx.logfd);

	if (new_file_created) {
		update_metafile(&ctx.metafd);
	}
}

static void send_log_filelist()
{
	FRESULT ret = FR_OK;
	static uint8_t _time_data[48] = { 0 };

	ret = f_open(&ctx.metafd, "0://.metafile", FA_OPEN_EXISTING | FA_READ);
	if (ret != FR_OK) {
		ctx.sd_last_error = SD_RET_OPEN_ERR;
		send_to_external(CMD_SD_CARD, OP_SDCARD_ERROR, &ctx.sd_last_error, 1, 12, 32);
		return;
	}

	int dates = start_logfile_listing(&ctx.metafd);

	char buffer[64] = { 0 };
	uint16_t sequence_index = 0;
	uint8_t time_index = 0;
	/* char *n = NULL; */
	struct ymd ymd = { 0 };

	f_lseek(&ctx.metafd, 0);

	while (f_gets(buffer, 64, &ctx.metafd)) {

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

			/* DBG_LOG("header: "); */
			/* DBG_DUMP(_time_data, 5); */
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


	end_logfile_listing(&ctx.metafd);

ret_error:
	f_close(&ctx.metafd);
	/* return ret; */
}

static int start_logfile_listing(FIL *fil)
{
	int date_count = 0;
	char buffer[64] = { 0 };

	f_lseek(fil, 0);

	memset(fs_buffer, 0, sizeof(fs_buffer));

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
			/* DBG_LOG("header (%d): ", date_count); */
			/* for (int i = 0; i < date_count; i++) */
			/* 	DBG_DUMP(&fs_buffer[i * 4], 4); */
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

	/* if (t->Hours >= 24) { */
		DBG_LOG("%02d%02d%02d_%02d%02d%02d\r\n", d->Year, d->Month, d->Date,
			t->Hours, t->Minutes, t->Seconds);
	/* } */
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

static FRESULT write_log_file_header(FIL *fil)
{
	FRESULT res;
	uint16_t i;
	uni4Byte timezone;

	res = f_lseek(fil, 0x0000); // File 확장
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;

	for (i = 0; i < FILE_HEADER_SIZE; i++) {
		f_putc(0x00, fil); // File Header 초기화
	}

	res = f_lseek(fil, FILE_ADDR_FILE_CONFIRMATION); // File Confirmation
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	f_printf(fil, (const TCHAR *)"This is a TempMon File. devteam.");

	res = f_lseek(fil, FILE_ADDR_MODEL_NUMBER); // Model No.
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	f_printf(fil, "TTM-128");

	res = f_lseek(fil, FILE_ADDR_HW_VERSION); // H/W version
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	f_printf(fil, "1.10");

	res = f_lseek(fil, FILE_ADDR_FW_VERSION); // F/W version
		ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	f_printf(fil, "1.00");

	res = f_lseek(fil, FILE_ADDR_TEST_DATE_TIME); // Start Date Time
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	f_printf(fil, "20%02d%02d%02d%02d%02d%02d",
		SysTime.Date.Year, // file
		SysTime.Date.Month, SysTime.Date.Date,
		SysTime.Time.Hours, SysTime.Time.Minutes,
		SysTime.Time.Seconds);

	res = f_lseek(fil, FILE_ADDR_TIME_ZONE); // Time Zone
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	timezone.Float = 9;
	f_putc(timezone.UI8[0], fil);
	f_putc(timezone.UI8[1], fil);
	f_putc(timezone.UI8[2], fil);
	f_putc(timezone.UI8[3], fil);

	res = f_lseek(fil, FILE_ADDR_MCU_UUID); // MCU UUID
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
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

	res = f_lseek(fil, FILE_ADDR_SLOT_USE); // slot use
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	FOREACH(struct slot_s *s, ctx.slots) {
		f_putc(s->inserted, fil);
	}

	res = f_lseek(fil, FILE_ADDR_SLOT_TYPE); // slot type
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	FOREACH(struct slot_s *s, ctx.slots) {
		f_putc(s->type, fil);
	}

	res = f_lseek(fil, FILE_ADDR_NTC_TBL); // NTC Table
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;
	FOREACH(struct slot_s *s, ctx.slots) {
		UINT readnum = 0;
		res = f_write(fil, s->ntc.calibration_tbl,
			sizeof(float) * CHANNEL_NBR, &readnum);
	}

	res = f_lseek(fil, FILE_ADDR_TEST_DATA); // file write position
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_SEEK_ERR : SD_RET_OK;
	if (res != FR_OK)
		return res;

	return res;
}

static FRESULT write_data(FIL *fil)
{
	uint8_t sensorCount = 0;
	uint16_t writeLen = 0;
	UINT retCount;
	FRESULT res = FR_OK;
	// FSIZE_t size;

	//메모리 초기화
	memset(fs_data_buffer, 0x00, 420);

	//시간
	fs_data_buffer[0] = SysTime.Date.Year;
	fs_data_buffer[1] = SysTime.Date.Month;
	fs_data_buffer[2] = SysTime.Date.Date;
	fs_data_buffer[3] = SysTime.Time.Hours;
	fs_data_buffer[4] = SysTime.Time.Minutes;
	fs_data_buffer[5] = SysTime.Time.Seconds;
	fs_data_buffer[6] = (uint8_t)((SysTime.Time.SubSeconds & 0x0000FF00) >> 8);
	fs_data_buffer[7] = (uint8_t)(SysTime.Time.SubSeconds & 0x000000FF);

	//채널
	FOREACH(struct slot_s *s, ctx.slots) {
		if (!s || !s->inserted)
			continue;

		for (int i = 0; i < 16; i++) {
			if (s->ntc.channel_states[i] != CHANNEL_STATE_DISCONNECTED) {
				fs_data_buffer[34 + (sensorCount * 6)] = s->id * 16 + i; // 채널 번호
				fs_data_buffer[35 + (sensorCount * 6)] = s->ntc.channel_states[i]; // 센서 상태
				*((float *)&fs_data_buffer[36 + (sensorCount * 6)]) = s->ntc.temperatures[i]; // 센서값 저장
				sensorCount++; // 사용 채널수 확인
			}
		}
	}
	/* for (int j = 0; j < 4; j++) { */
	/*     for (int i = 0; i < 16; i++) { */
	/*         if (TestData.sensorState[j][i] != LDM_DONOT_CONNECT) { */
	/*             fs_data_buffer[34 + (sensorCount * 6)] = j * 16 + i; // 채널 번호 */
	/*             fs_data_buffer[35 + (sensorCount * 6)] = */
	/*                 TestData.sensorState[j][i]; // 센서 상태 */
	/*             memcpy(&fs_data_buffer[36 + (sensorCount * 6)], */
	/*                    &TestData.temperatures[j][i], 4); // 센서값 저장 */
	/*             sensorCount++; // 사용 채널수 확인 */
	/*         } */
	/*     } */
	/* } */

	if (sensorCount == 0) //센서가 모두 연결 안되있을때는 통과
	{
		return res;
	}

	fs_data_buffer[8] = sensorCount;

	//보드 센서 복사
	__IO float v;
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

		fs_data_buffer[10 + (i * 6)] = MCU_BOARD_BATTERY + i; //채널 번호, 0xEB ~ 0xEE
		fs_data_buffer[11 + (i * 6)] = 0x00; //보드 센서의 상태값은 없음.
		*((float *)&fs_data_buffer[12 + (i * 6)]) = v;
	}

	res = f_lseek(fil, f_size(fil));
	if (res != FR_OK) {
		ctx.sd_last_error = SD_RET_SEEK_ERR;
		return res;
	} else {
		ctx.sd_last_error = SD_RET_OK;
	}

	writeLen = (sensorCount * 6) + 34;
	res = f_write(fil, &fs_data_buffer[0], writeLen, &retCount);
	ctx.sd_last_error = (res != FR_OK) ? SD_RET_WRITE_ERR : SD_RET_OK;
	if (res != FR_OK) {
		return res;
	}

	return res;
}

static FRESULT update_metafile(FIL *fil)
{
	char buffer[64] = { 0 };
	FRESULT ret = FR_OK;

	ret = f_open(fil, "0://.metafile", FA_OPEN_APPEND | FA_READ | FA_WRITE);
	if (ret != FR_OK) {
		DBG_LOG("error (%d): f_open metafile during %s\n", ret, __func__);
		return ret;
	}

	/* move pointer to last line & read the line */
	if (f_size(fil) == 0) {
		if (SysTime.Date.Year >= 20) {
			f_printf(fil, "%02d%02d%02d\n",
				SysTime.Date.Year, SysTime.Date.Month, SysTime.Date.Date);
			f_printf(fil, "\t%s\n", current_log_filename());
		}
	} else {
		f_lseek(fil, f_size(fil) - 20);
		f_gets(buffer, 19, fil);

		if (buffer[0] != '\t') {
			DBG_LOG("error: can not find last line of metafile\n");
			f_close(fil);
			return 100;
		}

		struct ymd last_ymd = { 0 };
		translate_ymd(&last_ymd, &buffer[1]);
		if (last_ymd.d != SysTime.Date.Date) {
			f_lseek(fil, f_size(fil));
			f_printf(fil, "%02d%02d%02d\n",
				SysTime.Date.Year, SysTime.Date.Month, SysTime.Date.Date);
		}

		char *n = strndup(&buffer[8], 2);
		uint8_t hour = (uint8_t)strtol(n, NULL, 10);
		free(n);
		if (hour != SysTime.Time.Hours) {
			f_lseek(fil, f_size(fil));
			f_printf(fil, "\t%s\n", current_log_filename());
		}
	}

	f_sync(fil);
	f_close(fil);

	return ret;
}

static int send_log_file(const char *filename, bool is_header)
{
	FRESULT ret = FR_OK;

	ctx.heavy_job_processing = true;

	ret = f_open(&downloadfp, filename, FA_OPEN_EXISTING | FA_READ);
	ctx.sd_last_error = (ret != FR_OK) ? SD_RET_OPEN_ERR : SD_RET_OK;
	if (ret != FR_OK) {
		DBG_LOG("error (%d): f_open download during %s with %s\n",
			ret, __func__, filename);
		ctx.heavy_job_processing = false;
		return -1;
	}

	if (is_header) {
		send_file_header(&downloadfp);
	} else {
		send_file_overall(&downloadfp);
	}

	f_close(&downloadfp);

	ctx.heavy_job_processing = false;

	return 0;
}

static void send_file_header(FIL *fil)
{
	FRESULT ret = FR_OK;

	/* 1. starting packet */
	uint32_t filesize = f_size(fil);
	/* send a starting packet */
	uint32_t data[2] = { 0 };
	data[0] = filesize;
	data[1] = FILE_CHUNCK_SIZE;
	_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_START | 0xF0, data, 8, 12);

	/* 2. send chunks */
	UINT readnum = 0;
	UINT totalnum = 0;
	uint32_t count = 0;

	memset(fs_data_buffer, 0, sizeof(fs_data_buffer));

	ret = f_read(fil, &fs_data_buffer[4], FILE_CHUNCK_SIZE, &readnum);
	if (ret != FR_OK) {
		f_close(fil);
		return;
	}

	/* send packets */
	*((uint32_t *)&fs_data_buffer[0]) = count;
	_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_BODY | 0xF0, fs_data_buffer, readnum + 4, readnum + 4);

	readnum = 0;
	*((uint32_t *)&fs_data_buffer[0]) = readnum;
	memset(fs_data_buffer, 0, sizeof(fs_data_buffer));
	_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_END | 0xF0, fs_data_buffer, readnum + 4, readnum + 4);
	DBG_LOG("%s: %d readnum (%u), totalnum (%u)\r\n", __func__, count, readnum, totalnum);
}

static void send_file_overall(FIL *fil)
{
	FRESULT ret = FR_OK;

	/* 1. starting packet */
	uint32_t filesize = f_size(fil);
	/* send a starting packet */
	uint32_t data[2] = { 0 };
	data[0] = filesize;
	data[1] = FILE_CHUNCK_SIZE;
	_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_START, data, 8, 12);

	/* 2. send chunks */
	UINT readnum = 0;
	UINT totalnum = 0;
	uint32_t count = 0;
	while (!f_eof(fil)) {
		readnum = 0;
		memset(fs_data_buffer, 0, sizeof(fs_data_buffer));

		ret = f_read(fil, &fs_data_buffer[4], FILE_CHUNCK_SIZE, &readnum);
		if (ret != FR_OK) {
			f_close(fil);
			return;
		}

		/* send packets */
		if (readnum < FILE_CHUNCK_SIZE) {
			*((uint32_t *)&fs_data_buffer[0]) = readnum;
			_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_END, fs_data_buffer, readnum + 4, readnum + 4);
		} else {
			*((uint32_t *)&fs_data_buffer[0]) = count;
			_send_fs_packet(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_BODY, fs_data_buffer, readnum + 4, readnum + 4);
		}
		count++;
		totalnum += readnum;
		DBG_LOG("%s: %d readnum (%u), totalnum (%u)\r\n", __func__, count, readnum, totalnum);
	}

	/* send ending packet */
}

static void _send_fs_packet(uint8_t cmd, uint8_t option, void *data, size_t datasize, size_t arraysize)
{
	doMakeSend485Data(fs_buffer, cmd, option, data, datasize, arraysize);
	/* int ftxsize = fill_external_tx_frame(tx_buffer, ftx->cmd, ftx->option, */
	/* 				ftx->ipaddr, ftx->datetime, ftx->data, ftx->len - 20); */
	/* DBG_DUMP(fs_buffer, arraysize + 20); */

	while (!ext_tx_completed)
		__NOP();

	ext_tx_completed = 0;
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, fs_buffer, arraysize + 20);
	/* while (ext_tx_completed == 0) */
	/* 	__NOP(); */

}

static int delete_log_file(const char *filename)
{
	FRESULT ret = FR_OK;

	ret = f_unlink(filename);
	if (ret != FR_OK) {
		ctx.sd_last_error = SD_RET_REMOVE_ERR;
		DBG_LOG("error (%d): f_unlink %s with %s\n",
			ret, __func__, filename);
		return -1;
	}
	return 0;
}
