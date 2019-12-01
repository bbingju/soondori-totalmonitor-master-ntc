#ifndef __GLOBAL_DEFINE_H__
#define __GLOBAL_DEFINE_H__


/*********************************************************************
*		System Infomation
**********************************************************************/
#define STEP_SLOT_ID        0
#define STEP_READ_THRESHOLD 1
#define STEP_TEMP_READ      2

#define MCU_BOARD_BATTERY	0xEB
#define MCU_BOARD_RTD		0xEC
#define MCU_BOARD_TEMP	 	0xED
#define MCU_BOARD_HUMI  	0xEE
#define SD_CARD_FULL_ERROR_RATE ((float)0.1)

/*********************************************************************
*		RS485
**********************************************************************/
#define UART_RS485_HANDEL 		huart1

#define RS_485_TX_BUF_MAX 		100
#define RS_485_RX_BUF_MAX 		100
#define MAX_485_BUF_LEN			(16 * 1024 + 50)

/*********************************************************************
*		to Slot Uart
**********************************************************************/
#define UART_SLOT_HANDEL 		    huart2

#define UART_TX_BUF_MAX 			20
#define UART_RX_BUF_MAX 			3000

/*********************************************************************
*		File Header
**********************************************************************/
#define	FILE_HEADER_SIZE			4096

#define	FILE_ADD_FILE_CONFIRMATION	0x0000
#define	FILE_ADD_MODEL_NUMBER		0x0020
#define	FILE_ADD_HW_VERSION			0x0040
#define	FILE_ADD_FW_VERSION			0x0050
#define	FILE_ADD_TEST_DATE_TIME		0x0060
#define	FILE_ADD_TIME_ZONE			0x0070
#define	FILE_ADD_MCU_UUID			0x0080
#define	FILE_ADD_SLOT_USE			0x0090
#define	FILE_ADD_SLOT_TYPE			0x00A0
#define	FILE_ADD_FINGERPRINT		0x00B0
#define	FILE_ADD_BOARD_ID			0x00C0
#define	FILE_ADD_BOARD_IP			0x00D0

#define	FILE_ADD_TEST_DATA			0x1000

/*********************************************************************
*		Bluetooth
**********************************************************************/
#define BLUETOOTH_HANDEL 		huart2

#define TRUE				    1
#define FALSE					0

#define READY					0
#define BUSY					1

#define RATE_9600				9600
#define RATE_921600				921600

#define STANDBY					2
#define CONNECT					3

#define BT_RX_DMA_SIZE 			16
//#define BT_STX 				0x7F
//#define BT_ETX 				0x7E

#define CMD_STATUS				0xAA
#define CMD_LIST				0xC1
#define CMD_DOWNLOAD			0xC2
#define CMD_START				0xD1
#define CMD_STOP				0xD2
#define CMD_SAMPLERATE			0xA1
#define CMD_ACCELERATION_RANGE	0xA2
#define CMD_ENCODER_SCALE		0xA4
#define CMD_TIMEOUT				0xA8
#define CMD_RESET				0xB1
#define CMD_BEEP				0xB2
#define CMD_INFO				0xB4
#define CMD_TXDELAY				0xB8
//#define CMD_SD_CARD			0xE0

#define OPT_NULL				0x00

// CMD_INFO						0xB4
#define OPT_INFO_MODEL_NO		0x00
#define OPT_INFO_HW_VERSIO		0x01
#define OPT_INFO_FW_VERSION		0x02

// CMD_LIST						0xC1
#define OPT_LIST_DEFULT			0x00
#define OPT_LIST_START			0x01
#define OPT_LIST_END			0x02

// CMD_DOWNLOAD					0xC2
#define OPT_DOWN_FILE			0x00
#define OPT_DOWN_HEADER			0xFF
#define OPT_DOWN_START			0x01
#define OPT_DOWN_END			0x02

// CMD_START					0xD1
#define OPT_START_NO_SAVE		0xCA
#define OPT_START_RETURN_NAME	0x01
#define OPT_START_TEST			0xDA

// CMD_RANGE					0xA2
#define OPT_RANGE_2G			0x03
#define OPT_RANGE_4G			0x05
#define OPT_RANGE_8G			0x08
#define OPT_RANGE_16G			0x0C

// CMD_BEEP						0xB2
#define OPT_BEEP_OFF			0x00
#define OPT_BEEP_ON				0x01
#define OPT_BEEP_PATTERN		0x02

//	CMD_SDCARD
#define OPT_SDCARD_FORMAT		0x00



extern void RESET_USART2_Init(void);

/*********************************************************************
*		0_StartGetSensorTask.c
**********************************************************************/

#define SENSOR_BUF_MAX			5380	//3축 + endcoder + switch = 8byte * 32개(16msec) * 21회 = 5376byte(336msec) 이며, 5376 + 4 로 버퍼 생성 
#define SENSING_STOP 			0
#define SENSING_START   		1
#define SENSING_SAVE			2
#define SENSING_START_SAVE		4

/*********************************************************************
*		0_Util.c
**********************************************************************/
//extern uint8_t StringCopyToArray(uint8_t* TagetArray, uint8_t* OriginalString, uint8_t CopyLength, uint8_t TotalLength);


#endif

