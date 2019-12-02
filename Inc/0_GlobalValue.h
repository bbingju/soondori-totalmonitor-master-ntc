#ifndef __GLOBAL_VALUE_H__
#define __GLOBAL_VALUE_H__
//#endif 

#ifndef __STM32F4xx_HAL_SD_H
  //#include "stm32f4xx_hal_sd.h"
#endif 

#ifndef _STDINT
	#include "stdint.h"
#endif

#ifndef __STM32F4xx_HAL_UART_H
	#include "stm32f4xx_hal_uart.h"
#endif

/**********************************************************
* 변수는 소문자로 시작
* struct 변수는 대문자로 시작 
* struct는 대문자로만 작성 
* 함수는 대문자로 시작
* enum 은 대문자로만 작성 
***********************************************************/

#include "ff.h"
#include "main.h"
#include "0_GlobalDefine.h"

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
extern osSemaphoreId BinarySemSlaveRxHandle;
extern osSemaphoreId BinarySemSlaveTxHandle;
extern osSemaphoreId BinarySem485RxHandle;
extern osSemaphoreId CountingSem485TxHandle;
extern osSemaphoreId CountingSemSlaveRxHandle;

typedef union {
	float 		Float;
	uint32_t	UI32;
	uint16_t 	UI16[2];
	uint8_t 	UI8[4];	
	int16_t 	SI16[2];
	int8_t 		SIi8[4];	
}uni4Byte;

typedef union{
	uint16_t	UI16;
	uint8_t		UI8[2];
}uni2Byte;

/*********************************************************************
*	Test Data
* Main board 와 Sensor board 에서 읽어낸 데이터 저장 공간 
**********************************************************************/
typedef enum {
	MBS_BATTERY = 0,
	MBS_RTD,
	MBS_TEMP,
	MBS_HUMI	
}MAINBOARDSENSOR;

typedef enum {
	DT_FLOAT = 0,
	DT_UINT16,
	DT_UINT8,
	DT_INT16,
	DT_INT8
}DATATYPE;

typedef enum {
	SBT_NTC = 1,
	SBT_RELAY,
	SBT_RTD,
	SBT_MULTI,
	SBT_BLUETOOTH
}SENSORBOARDTYPE;

typedef enum {
	RSPI_OK = 0,
	RSPI_ERROR
}RETURNSPI; 

typedef enum {
	LDM_NORMAL_TEMP = 0,	//정상
	LDM_OVER_TEMP,			//경고온도 초과
	LDM_SENSOR_CLOSE,		//센서 쇼트
	LDM_DONOT_CONNECT		//센서 없음 
}LED_DIPLAY_MODE;

typedef struct 
{
	uni4Byte		mainBoard[4];				// MAINBOARDSENSOR 사용 해서 선택 
	uint32_t		mainBoardADC[4];			// main board 내부 센서 adc 값 
	uni4Byte		temperature[4][32];			// adc 완료후 온도값으로 환산된 값, 0 : 피대상물 온도, 1 : 환경온도 , 0~15채널 
	uni4Byte		adcMidValue[4][32];			// 컨버팅 완료된 ADC 값중 중간 값,  0 : 피대상물 온도, 1 : 환경온도, 0~15채널 
	uni4Byte		threshold[4][32];			// 경고 온도 저장 
	LED_DIPLAY_MODE	sensorState[4][32];			// 센서 상태 저장용 
	uni4Byte		ntcCalibrationTable[4][32];	// NTC 교정상수 테이블, RTD - NTC 로 계산되는 상수 
	uni4Byte		rtdCalibrationConst;		// RTD 교정상수 
	uni4Byte		ntcCalibrationConst;		// NTC 증감 상수, NTC board 에서 NTC + ntcCalibration + ntcCalibrationConst 해서 NTC 값이 생성됨 
	uint8_t			revisionApply[4];			// 보정 적용 상태, 1: 표면온도모드(보정 적용) 0 : 측정온도 모드(보정 미적용)
	uni4Byte		revisionConstant[4];		// 보정용 접촉상수 저장 
}TEST_DATA;
extern TEST_DATA TestData;

typedef struct
{
	uint8_t readTemp;									//응답이 왔는지 확인 하는 플래그 
}UPDATA_FLAG;
extern UPDATA_FLAG Updata_Flag;

/*********************************************************************
*	System Properties
* 시스템에서 공용으로 사용 되는 변수 저장  
**********************************************************************/
typedef enum {
	DPM_NORMAL = 0,
	DPM_TEMP_ERROR,
	DPM_SDCARD_ERROR,
	DPM_SETTING
}DISPLAYMODE;

typedef struct 
{						
    uint8_t         bootingWate[4];     //각 바이트에 Task별 부팅 완료를 표시 한다. 
                                        // 0 : StartDiaplayTask, 1 : StartRs485Task, 2 : StartSlotUartTask, 3 : StartRateTask
    DISPLAYMODE 	displayMode;        // DISPLAYMODE 사용해서 선택, 
    									// NORMAL MODE 에서는 -, \, |, / 을 순차적으로 표시 한다. 
    									// SETTING MODE 에서는 설정 값들을 변경 한다.
    uint8_t         slotType[4];        // SENSORBOARDTYPE 사용 해서 선택, sensor board 종류 기록 
    uint8_t         slotInsert[4];      // slot 삽입됬는지 확인 
    uni4Byte		intervalTime;       // sample rate 를 만들기 위한 task delay time 설정 
    uint8_t         start_flag;         // 온도 실시간 감시 플레
	uint8_t 		InterfaceStep;		// 초기화 단계 표시용 
	uni4Byte		mcuUUID[3];			// MCU UUID
	uint8_t			smpsState;			// smps 신호 확인 
}SYSTEM_STRUCT;
extern SYSTEM_STRUCT SysProperties;

typedef struct 
{		
    RTC_DateTypeDef Date;
    RTC_TimeTypeDef Time;
}SYSTEM_TIME;
extern SYSTEM_TIME  SysTime;


/*********************************************************************
*	Button 처리용 
**********************************************************************/
typedef enum {
	BTN_NORMAL = 0,
	BTN_FALLING,
	BTN_ENTERED
}BUTTONMODE;

/*********************************************************************
*		RS485
**********************************************************************/
typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	  ar[RS_485_TX_BUF_MAX];
}RS_485_TX_QUEUE_STRUCT;
extern RS_485_TX_QUEUE_STRUCT Rs485TxQueue;

typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	  ar[RS_485_RX_BUF_MAX];
}RS_485_RX_QUEUE_STRUCT;
extern RS_485_RX_QUEUE_STRUCT Rs485RxQueue;

typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	  ar[UART_TX_BUF_MAX];
}TX_QUEUE_STRUCT;
extern TX_QUEUE_STRUCT TxQueue;

typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	  ar[UART_RX_BUF_MAX];
}RX_QUEUE_STRUCT;
extern RX_QUEUE_STRUCT RxQueue;

//////////////////////////////////////////////////////////////////
//	SD CARD
//////////////////////////////////////////////////////////////////
typedef enum {
	SCS_OK = 0,
	SCS_LINK_ERROR,
	SCS_MOUNT_ERROR,
	SCS_MKDIR_ERROR,
	SCS_OPEN_ERROR,
	SCS_SEEK_ERROR,
	SCS_READ_ERROR,
	SCS_SYNC_ERROR,
	SCS_WRITE_ERROR,
	SCS_CLOSE_ERROR,
	SCS_READDIR_ERROR,
	SCS_OPENDIR_ERROR,
	SCS_DISK_FULL
}SD_CARD_STATE;

typedef struct 
{
	FATFS			sdFatFs;
	const TCHAR* 	path;
	FIL 			fileObject; 			// File object 
	TCHAR			loadFileName[40];
	char			diskPath[4];			// User logical drive path 
	SD_CARD_STATE	sdState;				// 에러 체크용
	SD_CARD_STATE 	sdMountState;
	FILINFO			fno;					// 파일 정보 리턴용 

	DIR				scanDir[5];				// 파일 리스트 전송용
	FIL 			sendFileObject; 
	uint8_t			scanDirDeep;
	uint8_t			scanFilePath[50];
	uint32_t		scanFileListCount;
	uint8_t			scanReadFileName[50];

	uint8_t 		sendFileName[50];		// 파일 다운로드용 
	uint8_t			sendFileNameLen;
	
	//uint8_t			errorState;		// 에러 체크용
}SD_CARD_VALUE;
extern SD_CARD_VALUE	sdValue;

extern uint8_t			FindFilelistFlag;		//0 : 파일 리스트 검색 안하는중 / 1 : 파일 리스트 검색중 

extern GPIO_TypeDef *	SLAVE_CS_PORT[4];
extern uint16_t			SLAVE_CS_PIN[4];

//extern uint8_t 		rxDataBuffer[UART_RX_BUF_MAX];
//extern uint8_t 		txDataBuffer[UART_TX_BUF_MAX];
extern uint8_t          sendSlotNumber;

extern uint8_t          rx485DataDMA[320];     			 	//dma 용도 
extern uint8_t			tx485DataDMA[MAX_485_BUF_LEN];      //dma 용도 
extern uint8_t 			ReadFileBuf[MAX_485_BUF_LEN];

#endif
