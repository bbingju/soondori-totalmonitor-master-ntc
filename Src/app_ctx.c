#include "app_ctx.h"
#include "debug.h"

#include <stdio.h>
#include <string.h>

FRESULT ff_create_metafile(FIL *mf, char *path)
{
	DIR dir;
	static FILINFO fno;
	FRESULT ret;
	static int depth = 0;
	char tempbuf[48] = { 0 };

	if (depth > 3)
		return FR_TOO_MANY_OPEN_FILES;

	ret = f_opendir(&dir, path);
	if (ret) {
		DBG_LOG("error (%d): Can't open dir %s (%d depth)\n", ret, path, depth);
		return ret;
	}

	depth++;

	while (1) {
		ret = f_readdir(&dir, &fno);
		if (ret || !fno.fname[0])
			break;

		if (fno.fattrib & AM_DIR) {
			strcpy(tempbuf, path);
			sprintf(&tempbuf[strlen(path)], "/%s", fno.fname);
			DBG_LOG("<dir> %s\n", tempbuf);
			if (depth == 3) {
				f_putc(tempbuf[6], mf);
				f_putc(tempbuf[7], mf);
				f_putc(tempbuf[9], mf);
				f_putc(tempbuf[10], mf);
				f_putc(tempbuf[12], mf);
				f_putc(tempbuf[13], mf);
				f_putc('\n', mf);
			}
			if (tempbuf[6] == '2' && tempbuf[7] == '0') {
				ret = ff_create_metafile(mf, tempbuf);
				if (ret != FR_OK) break;
			}
		} else {
			DBG_LOG("\t%s/%s\n", path, fno.fname);
			if (fno.fname[0] != '.') {
				f_printf(mf, "\t%s\n", &fno.fname[7]);
			}
		}
	}

	depth--;
	f_closedir(&dir);

	f_sync(mf);
	return ret;
}

FRESULT ff_enumerate_dir(char *path)
{
	DIR dir;
	static FILINFO fno;
	FRESULT ret;
	int i;
	static int depth = 0;
	char tempbuf[128] = { 0 };

	if (depth > 4)
		return FR_TOO_MANY_OPEN_FILES;

	ret = f_opendir(&dir, path);
	if (ret) {
		DBG_LOG("error (%d): Can't open dir %s (%d depth)\n", ret, path, depth);
		return ret;
	}

	depth++;

	while (1) {
		ret = f_readdir(&dir, &fno);
		if (ret || !fno.fname[0])
			break;

		if (fno.fattrib & AM_DIR) {
			strcpy(tempbuf, path);
			i = strlen(path);
			sprintf(&tempbuf[i], "/%s", fno.fname);
			DBG_LOG("<dir> %s\n", tempbuf);
			ret = ff_enumerate_dir(tempbuf);
			if (ret != FR_OK) break;
		} else {
			DBG_LOG("\t%s/%s\n", path, fno.fname);
		}
	}
	depth--;
	f_closedir(&dir);

	return ret;
}

void app_ctx_init(app_ctx_t *ctx)
{
	if (!ctx)
		return;

	/* initialize FatFS */
	MX_FATFS_Init();
	if (retSD != 0) {
		DBG_LOG("error: MX_FATFS_Init()\n");
	}

	ctx->sd_ff = &SDFatFS;
	ctx->sd_root = SDPath;

	DBG_LOG("Now mount drive %s\n", ctx->sd_root);
	FRESULT ret = f_mount(ctx->sd_ff, (const TCHAR *)ctx->sd_root, 0);
	if (ret != FR_OK) {
		DBG_LOG("error (%): Can't not mount to %s\n", ret, ctx->sd_root);
	}

	uint32_t start = osKernelSysTick();

	FIL mfile;
	ret = f_open(&mfile, "0://.metafile", FA_CREATE_ALWAYS | FA_WRITE);
	if (ret) {
		DBG_LOG("error (%d): f_open()\n", ret);
	}
	ff_create_metafile(&mfile, ctx->sd_root);
	/* ff_enumerate_dir(ctx->sd_root); */
	f_close(&mfile);
	DBG_LOG("elapsed ticks: %u\n", osKernelSysTick() - start);
}
