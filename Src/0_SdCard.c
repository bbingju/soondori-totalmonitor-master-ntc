#include <stdio.h>
#include <string.h>

#include "0_SdCard.h"
#include "0_GlobalValue.h"
#include "external_uart_task.h"
#include "debug.h"

uint8_t 	wData[420];

/*********************************************************************
*	MountSDIO
*	SD CARD 마운트 함수
**********************************************************************/
FRESULT MountSDIO(void)
{
	/* FRESULT res = f_mount(&sdValue.sdFatFs, (const TCHAR *)SDPath, 0); */
	FRESULT res = f_mount(&SDFatFS, (const TCHAR *)SDPath, 0);
	if (res != FR_OK) {
	} else {
	}
	return res;
}

/*********************************************************************
*	UnMountSDIO
*	SD CARD 마운트 해제 함수
**********************************************************************/
FRESULT UnMountSDIO(void)
{
	FRESULT res = f_mount(NULL, (const TCHAR *)"", 1);
	if (res != FR_OK) {
	} else {
	}
	return res;
}

static FRESULT check_n_create_directory(const TCHAR *dirname)
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

/*********************************************************************
*	DoFolderCheck
*	현재 시간을 기준으로 해당 폴더가 있는지 확인 하여 폴더를 생성 한다.
**********************************************************************/
FRESULT DoFolderCheck(void)
{
	TCHAR dirname[16] = {0};
	FRESULT res = FR_OK;
	RTC_DateTypeDef *date = &SysTime.Date;

	sprintf(dirname, "20%02d", date->Year);
	res = check_n_create_directory(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

	sprintf(dirname, "20%02d/%02d", date->Year, date->Month);
	res = check_n_create_directory(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

	sprintf(dirname, "20%02d/%02d/%02d", date->Year, date->Month, date->Date);
	res = check_n_create_directory(dirname);
	if (res != FR_OK) {
		DBG_LOG("%s: Error to create a directory!\n", __func__);
		goto ret;
	}

ret:
	return res;
}

/*********************************************************************
*	DoFileCheck
*	이미 열려 있는 파일이 있다면 닫은후 작업 한다.
*	확인후 파일을 생성 한다.
**********************************************************************/
void DoFileCheck(void)
{
	if (sdValue.loadFileName[0] == 0) { //열려 있는 파일이 없을 경우
		DoMakeLoadFileName();
		DoMakeFile();
	} else {		//열려있는 파일이 있는 경우
		if ((sdValue.loadFileName[2] == SysTime.Date.Year / 10 + '0') &&
			(sdValue.loadFileName[3] == SysTime.Date.Year % 10 + '0') &&
			(sdValue.loadFileName[5] == SysTime.Date.Month / 10 + '0') &&
			(sdValue.loadFileName[6] == SysTime.Date.Month % 10 + '0') &&
			(sdValue.loadFileName[8] == SysTime.Date.Date / 10 + '0') &&
			(sdValue.loadFileName[9] == SysTime.Date.Date % 10 + '0') && //날짜 비교
			(sdValue.loadFileName[18] == SysTime.Time.Hours / 10 + '0') &&
			(sdValue.loadFileName[19] == SysTime.Time.Hours % 10 + '0')) { //시간 확인
			osDelay(1);
		} else { //시간이 변경된 경우
			DoFileClose();
			DoMakeLoadFileName();
			DoMakeFile();
		}
	}
}

/*********************************************************************
*	DoMakeFile
*	생성된 파일이 있는지 확인한 후 파일을 만든다.
*	파일은 현재 시간을 기준으로 시간 단위가 변경되면 새로 만든다.
*	시간이 같으면 기존 파일의 마지막에 이어서 쓴다.
**********************************************************************/
void DoMakeFile(void)
{
    FRESULT res = FR_OK;

    res = f_stat(sdValue.loadFileName, &sdValue.fno);
    if (res != FR_OK) //파일이 없으면
    {
        res = f_open(&sdValue.fileObject, (const TCHAR *)sdValue.loadFileName,
                     FA_OPEN_ALWAYS | FA_WRITE);
        if (res != FR_OK) //파일 열기 오류
        {
            sdValue.sdState = SCS_OPEN_ERROR;
            send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                                   (uint8_t *)sdValue.sdState, 1, 12, 32);
            return;
        } else {
            sdValue.sdState = SCS_OK;
        }
        DoWriteFileHeader();
    } else //파일이 있을때,  부팅을 했는데 예전 파일이 이름이 같을때
    {
        res = f_open(&sdValue.fileObject, (const TCHAR *)sdValue.loadFileName,
                     FA_OPEN_APPEND | FA_WRITE); //이어 쓰기 한다.
        if (res != FR_OK)                        //파일 열기 오류
        {
            sdValue.sdState = SCS_OPEN_ERROR;
            send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                                   (uint8_t *)sdValue.sdState, 1, 12, 32);
            return;
        } else {
            sdValue.sdState = SCS_OK;
        }
    }
}

/*********************************************************************
*	DoFileClose
*	파일은 닫고 파일 이름을 클리어 하여 열린 파일이 없다고 표시 한다.
**********************************************************************/
void DoFileClose(void)
{
    f_close(&sdValue.fileObject);
    memset(sdValue.loadFileName, 0x00, sizeof(sdValue.loadFileName));
}

/*********************************************************************
*	DoMakeLoadFileName
*	시간으로 파일 이름을 생성 한다.
**********************************************************************/
void DoMakeLoadFileName(void) //현제시간으로 파일 만들기
{
    sprintf(sdValue.loadFileName,
            "20%02d/%02d/%02d/%02d%02d%02d_%02d%02d%02d.ske",
            SysTime.Date.Year, // dir
            SysTime.Date.Month, SysTime.Date.Date,
            SysTime.Date.Year, // file
            SysTime.Date.Month, SysTime.Date.Date, SysTime.Time.Hours,
            SysTime.Time.Minutes, SysTime.Time.Seconds);
}

/*********************************************************************
*	DoWriteFileHeader
*	파일 해더 작성
**********************************************************************/
void DoWriteFileHeader(void)
{
    FRESULT res;
    uint16_t i;
    // uint8_t 	temp[4];
    uni4Byte timezone;

    res = f_lseek(&sdValue.fileObject, 0x0000); // File 확장
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }

    for (i = 0; i < FILE_HEADER_SIZE; i++) {
        f_putc(0x00, &sdValue.fileObject); // File Header 초기화
    }

    res = f_lseek(&sdValue.fileObject,
                  FILE_ADD_FILE_CONFIRMATION); // File Confirmation
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_printf(&sdValue.fileObject,
             (const TCHAR *)"This is a TempMon File. devteam.");

    res = f_lseek(&sdValue.fileObject, FILE_ADD_MODEL_NUMBER); // Model No.
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_printf(&sdValue.fileObject, "TTM-128");

    res = f_lseek(&sdValue.fileObject, FILE_ADD_HW_VERSION); // H/W version
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_printf(&sdValue.fileObject, "1.10");

    res = f_lseek(&sdValue.fileObject, FILE_ADD_FW_VERSION); // F/W version
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_printf(&sdValue.fileObject, "1.00");

    res = f_lseek(&sdValue.fileObject,
                  FILE_ADD_TEST_DATE_TIME); // Start Date Time
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_printf(&sdValue.fileObject, "20%02d%02d%02d%02d%02d%02d",
             (char)SysTime.Date.Year, // file
             (char)SysTime.Date.Month, (char)SysTime.Date.Date,
             (char)SysTime.Time.Hours, (char)SysTime.Time.Minutes,
             (char)SysTime.Time.Seconds);
    res = f_lseek(&sdValue.fileObject, FILE_ADD_TIME_ZONE); // Time Zone
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    timezone.Float = 9;
    f_putc(timezone.UI8[0], &sdValue.fileObject);
    f_putc(timezone.UI8[1], &sdValue.fileObject);
    f_putc(timezone.UI8[2], &sdValue.fileObject);
    f_putc(timezone.UI8[3], &sdValue.fileObject);

    res = f_lseek(&sdValue.fileObject, FILE_ADD_MCU_UUID); // MCU UUID
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    f_putc(SysProperties.mcuUUID[0].UI8[0], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[0].UI8[1], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[0].UI8[2], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[0].UI8[3], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[1].UI8[0], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[1].UI8[1], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[1].UI8[2], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[1].UI8[3], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[2].UI8[0], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[2].UI8[1], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[2].UI8[2], &sdValue.fileObject);
    f_putc(SysProperties.mcuUUID[2].UI8[3], &sdValue.fileObject);

    res = f_lseek(&sdValue.fileObject, FILE_ADD_SLOT_USE); // slot use
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    for (int i = 0; i < 4; i++)
        f_putc(SysProperties.slots[i].inserted, &sdValue.fileObject);

    res = f_lseek(&sdValue.fileObject, FILE_ADD_SLOT_TYPE); // slot type
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
    for (int i = 0; i < 4; i++)
        f_putc(SysProperties.slots[i].type, &sdValue.fileObject);

    res =
        f_lseek(&sdValue.fileObject, FILE_ADD_TEST_DATA); // file write position
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }

    res = f_sync(&sdValue.fileObject);
    if (res != FR_OK) {
        sdValue.sdState = SCS_SYNC_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
}

/*********************************************************************
*	DoDataWrite
*	테스트 정보를 파일에 이어 쓰기 한다.
**********************************************************************/
void DoDataWrite(void)
{
    uint8_t i = 0;
    uint8_t sensorCount = 0;
    uint16_t writeLen = 0;
    UINT retCount;
    FRESULT res;
    // FSIZE_t size;

    //메모리 초기화
    memset(wData, 0x00, sizeof(wData));

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
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 16; i++) {
            if (TestData.sensorState[j][i] != LDM_DONOT_CONNECT) {
                wData[34 + (sensorCount * 6)] = j * 16 + i; // 채널 번호
                wData[35 + (sensorCount * 6)] =
                    TestData.sensorState[j][i]; // 센서 상태
                memcpy(&wData[36 + (sensorCount * 6)],
                       &TestData.temperatures[j][i], 4); // 센서값 저장
                sensorCount++; // 사용 채널수 확인
            }
        }
    }

    if (sensorCount == 0) //센서가 모두 연결 안되있을때는 통과
    {
        return;
    }

    wData[8] = sensorCount;

    //보드 센서 복사
    for (i = 0; i < 4; i++) {
        wData[10 + (i * 6)] = MCU_BOARD_BATTERY + i; //채널 번호, 0xEB ~ 0xEE
        wData[11 + (i * 6)] = 0x00; //보드 센서의 상태값은 없음.
        memcpy(&wData[12 + (i * 6)], &TestData.mainBoard[i].UI8[0],
               4); //센서값 저장
    }

    res = f_lseek(&sdValue.fileObject, f_size(&sdValue.fileObject));
    if (res != FR_OK) {
        sdValue.sdState = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }

    writeLen = (sensorCount * 6) + 34;
    res = f_write(&sdValue.fileObject, &wData[0], writeLen, &retCount);
    if (res != FR_OK) {
        sdValue.sdState = SCS_WRITE_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }

    res = f_sync(&sdValue.fileObject);
    if (res != FR_OK) {
        sdValue.sdState = SCS_SYNC_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR,
                               (uint8_t *)sdValue.sdState, 1, 12, 32);
        return;
    } else {
        sdValue.sdState = SCS_OK;
    }
}

/*********************************************************************
*	scan_files
*	파일 리스트 전송, 지정된 경로의 파일만 보낸다.
*	(파일과 폴더를 구분해서 보내지 못한다.)
*	path : 파일 리스트를 확인할 경로를 보낸다.
**********************************************************************/
#if 0
FRESULT scan_files(char *path) /* Start node to be scanned (***also used as work area***) */
{
    volatile FRESULT res;
    FILINFO fno;
    volatile int32_t i; //, namelen;
    volatile char *pc_fn;
    volatile uint8_t len1, len2;

#if _USE_LFN
    // char c_lfn[_MAX_LFN + 1];
    // fno.fname = c_lfn;
    // fno.lfsize = sizeof(c_lfn);
#endif

    /* Open the directory */
    res = f_opendir(&sdValue.scanDir[sdValue.scanDirDeep], path);
    /* osDelay(1); */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            memset(sdValue.scanFilePath, 0x00, sizeof(sdValue.scanFilePath));
            res = f_readdir(&sdValue.scanDir[sdValue.scanDirDeep], &fno);
            if (res != FR_OK || fno.fname[0] == 0) {
                // sdValue.sdState = SCS_READDIR_ERROR;
                /* osDelay(1); */
                break;
            }

            pc_fn = fno.fname;

            if (*pc_fn == '.') {
                /* osDelay(1); */
                continue;
            }

            if (fno.fattrib & AM_SYS) {
                /* osDelay(1); */
                continue;
            }

            for (len1 = 0; len1 < 50; len1++) {
                if (*(path + len1) == 0)
                    break;
                sdValue.scanFilePath[len1] = *(path + len1);
            }
            sdValue.scanFilePath[len1++] = '/';
            for (len2 = 0; len2 < 50; len2++) {
                if (*(pc_fn + len2) == 0)
                    break;
                sdValue.scanFilePath[len1 + len2] = *(pc_fn + len2);
            }

            if (fno.fattrib & AM_DIR) {
                sdValue.scanFilePath[len1 + len2++] = '>';
            }
	    sdValue.scanFilePath[len1 + len2] = 0;

            printf("%s\r", sdValue.scanFilePath);
            sdValue.scanFileListCount++;

            for (i = 0; i < len1 + len2 - 2; i++) {
                sdValue.scanFilePath[i] = sdValue.scanFilePath[i + 2];
            }
            /*			while(TxQueue_empty(&TxQueue) != TRUE)
                                    {
                                            //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
                                            osDelay(10);
                                    }*/
            sdValue.sdState = SCS_OK;
            send_external_response(CMD_SD_CARD, OP_SDCARD_LIST_BODY,
                                   (uint8_t *)sdValue.scanFilePath,
                                   len1 + len2 - 3, 36, 56);
        }
        printf("f_closedir %d\r",
               f_closedir(&sdValue.scanDir[sdValue.scanDirDeep]));
    } /*
     else
     {
             sdValue.sdState = SCS_OPENDIR_ERROR;
     }*/
    /* osDelay(1); */

    return res;
}
#endif	/* 0 */
