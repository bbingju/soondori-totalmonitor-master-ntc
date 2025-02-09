#ifndef _GLOBALVALUE_H
#define _GLOBALVALUE_H

#include "cmsis_os.h"
#include "fatfs.h"
#include "main.h"
#include "0_GlobalDefine.h"

#include <stdint.h>
#include <stdbool.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern RTC_HandleTypeDef hrtc;
extern SD_HandleTypeDef hsd;

extern osSemaphoreId myBinarySemModeHandle;
extern osSemaphoreId myBinarySemUpHandle;
extern osSemaphoreId myBinarySemDownHandle;

typedef __PACKED_UNION {
    float Float;
    uint32_t UI32;
    uint16_t UI16[2];
    uint8_t UI8[4];
    int16_t SI16[2];
    int8_t SIi8[4];
} uni4Byte;

typedef __PACKED_UNION {
    uint16_t UI16;
    uint8_t UI8[2];
} uni2Byte;

/*********************************************************************
*       Test Data
* Main board 와 Sensor board 에서 읽어낸 데이터 저장 공간
**********************************************************************/
/* typedef enum { */
/*     MBS_BATTERY = 0, */
/*     MBS_RTD, */
/*     MBS_TEMP, */
/*     MBS_HUMI */
/* } MAINBOARDSENSOR; */

typedef enum {
    DT_FLOAT = 0,
    DT_UINT16,
    DT_UINT8,
    DT_INT16,
    DT_INT8
} DATATYPE;

typedef enum {
    SBT_NTC = 1,
    SBT_RELAY,
    SBT_RTD,
    SBT_MULTI,
    SBT_BLUETOOTH
} SENSORBOARDTYPE;

typedef enum {
    RSPI_OK = 0,
    RSPI_ERROR
} RETURNSPI;

/* typedef enum { */
/*     LDM_NORMAL_TEMP = 0,    //정상 */
/*     LDM_OVER_TEMP,	    //경고온도 초과 */
/*     LDM_SENSOR_CLOSE,	    //센서 쇼트 */
/*     LDM_DONOT_CONNECT	    //센서 없음 */
/* } LED_DIPLAY_MODE; */

typedef __PACKED_STRUCT {
    /* uni4Byte mainBoard[4];    // MAINBOARDSENSOR 사용 해서 선택 */
    /* uint32_t mainBoardADC[4]; // main board 내부 센서 adc 값 */
    //uni4Byte adcMidValue[4][32]; // 컨버팅 완료된 ADC 값중 중간 값,  0 :
                                 // 피대상물 온도, 1 : 환경온도, 0~15채널
	//uni4Byte ntcCalibrationTable[4][32]; // NTC 교정상수 테이블, RTD - NTC 로
                                         // 계산되는 상수
    /* uni4Byte rtdCalibrationConst; // RTD 교정상수 */
    //uni4Byte ntcCalibrationConst; // NTC 증감 상수, NTC board 에서 NTC +
                                  // ntcCalibration + ntcCalibrationConst 해서
                                  // NTC 값이 생성됨
} TEST_DATA;

extern TEST_DATA TestData;

/*********************************************************************
*       System Properties
* 시스템에서 공용으로 사용 되는 변수 저장
**********************************************************************/
typedef enum {
    DPM_NORMAL = 0,
    DPM_TEMP_ERROR,
    DPM_SDCARD_ERROR,
    DPM_SETTING
} DISPLAYMODE;

typedef struct {
	/* DISPLAYMODE 사용해서 선택,
	   NORMAL MODE 에서는 -, \, |, / 을 순차적으로 표시 한다.
	   SETTING MODE 에서는 설정 값들을 변경 한다. */
	DISPLAYMODE displayMode;

	/* 초기화 단계 표시용 */
	uint8_t InterfaceStep;

	/* sample rate 를 만들기 위한 task delay time 설정 */
	uint32_t interval_ms;

	/* 온도 실시간 감시 플레 */
	uint8_t start_flag;

	uni4Byte mcuUUID[3]; // MCU UUID
	uint8_t smpsState;   // smps 신호 확인

	bool time_synced;
	FATFS *fatfs;
	bool sd_writing_available;

} SYSTEM_STRUCT;

extern SYSTEM_STRUCT SysProperties;

typedef struct {
    RTC_DateTypeDef Date;
    RTC_TimeTypeDef Time;
} SYSTEM_TIME;
extern SYSTEM_TIME SysTime;

/*********************************************************************
*       Button 처리용
**********************************************************************/
typedef enum { BTN_NORMAL = 0, BTN_FALLING, BTN_ENTERED } BUTTONMODE;

//////////////////////////////////////////////////////////////////
//      SD CARD
//////////////////////////////////////////////////////////////////
typedef enum {
    SD_RET_OK = 0,
    SD_RET_LINK_ERR,
    SD_RET_MOUNT_ERR,
    SD_RET_MKDIR_ERR,
    SD_RET_OPEN_ERR,
    SD_RET_SEEK_ERR,
    SD_RET_READ_ERR,
    SD_RET_SYNC_ERR,
    SD_RET_WRITE_ERR,
    SD_RET_CLOSE_ERR,
    SD_RET_READDIR_ERR,
    SD_RET_OPENDIR_ERR,
    SD_RET_DISKFULL_ERR,
    SD_RET_REMOVE_ERR,
} SD_RET_E;

typedef struct {
    /* FATFS sdFatFs; */
    /* const TCHAR *path; */
    /* FIL fileObject; // File object */
    /* TCHAR loadFileName[40]; */
    /* char diskPath[4];      // User logical drive path */
    /* SD_RET_E sdState; // 에러 체크용 */
    /* SD_RET_E sdMountState; */
    /* FILINFO fno; // 파일 정보 리턴용 */

    /* DIR scanDir[5]; // 파일 리스트 전송용 */
    /* FIL sendFileObject; */
    /* uint8_t scanDirDeep; */
    /* uint8_t scanFilePath[50]; */
    /* uint32_t scanFileListCount; */
    /* uint8_t scanReadFileName[50]; */

    /* uint8_t sendFileName[50]; // 파일 다운로드용 */
    /* uint8_t sendFileNameLen; */

    // uint8_t       errorState;             // 에러 체크용
} SD_CARD_VALUE;
extern SD_CARD_VALUE sdValue;

extern uint8_t   FindFilelistFlag;               //0 : 파일 리스트 검색 안하는중 / 1 : 파일 리스트 검색중

extern GPIO_TypeDef *   SLAVE_CS_PORT[4];
extern uint16_t         SLAVE_CS_PIN[4];

#endif /* _GLOBALVALUE_H */
