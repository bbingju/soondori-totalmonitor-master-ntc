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
* ������ �ҹ��ڷ� ����
* struct ������ �빮�ڷ� ���� 
* struct�� �빮�ڷθ� �ۼ� 
* �Լ��� �빮�ڷ� ����
* enum �� �빮�ڷθ� �ۼ� 
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
* Main board �� Sensor board ���� �о ������ ���� ���� 
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
	LDM_NORMAL_TEMP = 0,	//����
	LDM_OVER_TEMP,			//���µ� �ʰ�
	LDM_SENSOR_CLOSE,		//���� ��Ʈ
	LDM_DONOT_CONNECT		//���� ���� 
}LED_DIPLAY_MODE;

typedef struct 
{
	uni4Byte		mainBoard[4];				// MAINBOARDSENSOR ��� �ؼ� ���� 
	uint32_t		mainBoardADC[4];			// main board ���� ���� adc �� 
	uni4Byte		temperature[4][32];			// adc �Ϸ��� �µ������� ȯ��� ��, 0 : �Ǵ�� �µ�, 1 : ȯ��µ� , 0~15ä�� 
	uni4Byte		adcMidValue[4][32];			// ������ �Ϸ�� ADC ���� �߰� ��,  0 : �Ǵ�� �µ�, 1 : ȯ��µ�, 0~15ä�� 
	uni4Byte		threshold[4][32];			// ��� �µ� ���� 
	LED_DIPLAY_MODE	sensorState[4][32];			// ���� ���� ����� 
	uni4Byte		ntcCalibrationTable[4][32];	// NTC ������� ���̺�, RTD - NTC �� ���Ǵ� ��� 
	uni4Byte		rtdCalibrationConst;		// RTD ������� 
	uni4Byte		ntcCalibrationConst;		// NTC ���� ���, NTC board ���� NTC + ntcCalibration + ntcCalibrationConst �ؼ� NTC ���� ������ 
	uint8_t			revisionApply[4];			// ���� ���� ����, 1: ǥ��µ����(���� ����) 0 : �����µ� ���(���� ������)
	uni4Byte		revisionConstant[4];		// ������ ���˻�� ���� 
}TEST_DATA;
extern TEST_DATA TestData;

typedef struct
{
	uint8_t readTemp;									//������ �Դ��� Ȯ�� �ϴ� �÷��� 
}UPDATA_FLAG;
extern UPDATA_FLAG Updata_Flag;

/*********************************************************************
*	System Properties
* �ý��ۿ��� �������� ��� �Ǵ� ���� ����  
**********************************************************************/
typedef enum {
	DPM_NORMAL = 0,
	DPM_TEMP_ERROR,
	DPM_SDCARD_ERROR,
	DPM_SETTING
}DISPLAYMODE;

typedef struct 
{						
    uint8_t         bootingWate[4];     //�� ����Ʈ�� Task�� ���� �ϷḦ ǥ�� �Ѵ�. 
                                        // 0 : StartDiaplayTask, 1 : StartRs485Task, 2 : StartSlotUartTask, 3 : StartRateTask
    DISPLAYMODE 	displayMode;        // DISPLAYMODE ����ؼ� ����, 
    									// NORMAL MODE ������ -, \, |, / �� ���������� ǥ�� �Ѵ�. 
    									// SETTING MODE ������ ���� ������ ���� �Ѵ�.
    uint8_t         slotType[4];        // SENSORBOARDTYPE ��� �ؼ� ����, sensor board ���� ��� 
    uint8_t         slotInsert[4];      // slot ���ԉ���� Ȯ�� 
    uni4Byte		intervalTime;       // sample rate �� ����� ���� task delay time ���� 
    uint8_t         start_flag;         // �µ� �ǽð� ���� �÷�
	uint8_t 		InterfaceStep;		// �ʱ�ȭ �ܰ� ǥ�ÿ� 
	uni4Byte		mcuUUID[3];			// MCU UUID
	uint8_t			smpsState;			// smps ��ȣ Ȯ�� 
}SYSTEM_STRUCT;
extern SYSTEM_STRUCT SysProperties;

typedef struct 
{		
    RTC_DateTypeDef Date;
    RTC_TimeTypeDef Time;
}SYSTEM_TIME;
extern SYSTEM_TIME  SysTime;


/*********************************************************************
*	Button ó���� 
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
	SD_CARD_STATE	sdState;				// ���� üũ��
	SD_CARD_STATE 	sdMountState;
	FILINFO			fno;					// ���� ���� ���Ͽ� 

	DIR				scanDir[5];				// ���� ����Ʈ ���ۿ�
	FIL 			sendFileObject; 
	uint8_t			scanDirDeep;
	uint8_t			scanFilePath[50];
	uint32_t		scanFileListCount;
	uint8_t			scanReadFileName[50];

	uint8_t 		sendFileName[50];		// ���� �ٿ�ε�� 
	uint8_t			sendFileNameLen;
	
	//uint8_t			errorState;		// ���� üũ��
}SD_CARD_VALUE;
extern SD_CARD_VALUE	sdValue;

extern uint8_t			FindFilelistFlag;		//0 : ���� ����Ʈ �˻� ���ϴ��� / 1 : ���� ����Ʈ �˻��� 

extern GPIO_TypeDef *	SLAVE_CS_PORT[4];
extern uint16_t			SLAVE_CS_PIN[4];

//extern uint8_t 		rxDataBuffer[UART_RX_BUF_MAX];
//extern uint8_t 		txDataBuffer[UART_TX_BUF_MAX];
extern uint8_t          sendSlotNumber;

extern uint8_t          rx485DataDMA[320];     			 	//dma �뵵 
extern uint8_t			tx485DataDMA[MAX_485_BUF_LEN];      //dma �뵵 
extern uint8_t 			ReadFileBuf[MAX_485_BUF_LEN];

#endif
