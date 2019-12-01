#ifndef __START_RS485_TASK_H__
#define __START_RS485_TASK_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "0_Util.h"
#include "0_BCD110DS.h"

#ifndef __STM32F4xx_HAL_CONF_H
  #include "stm32f4xx_hal_conf.h"
#endif

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"
#include "0_soonQueue.h"

#define RS_READ_SIZE            			(32)

#define RS_STX 								(0x7F)
#define RS_ETX 								(0x7E)

#define	COMMAND		            			(5)
#define	OPTION		            			(6)
#define	DATA 			        			(7)

#define CMD_TEMP_TEST           			1
#define CMD_WARNING_TEMP        			2
#define CMD_REVISION            			3
#define CMD_CALIBRATION         			4
#define CMD_SD_CARD             			5
#define CMD_SLOT                			6
#define CMD_TIME                			7

#define OP_TEMP_START_RX           			1
#define OP_TEMP_MAIN_INFO					1
#define OP_TEMP_SLOT_INFO					2
#define OP_TEMP_CHANNEL_INFO     			3
#define OP_TEMP_CHANNEL_VALUE      			4
#define OP_TEMP_STOP            			5
#define OP_TEMP_SAMPLE_RATE     			6

#define OP_WARNING_TEMP_SET     			1
#define OP_WARNING_TEMP_REQ     			2

#define OP_REVISION_APPLY_SET      			1
#define OP_REVISION_CONSTANT_SET			2
#define OP_REVISION_APPLY_REQ      			3
#define OP_REVISION_CONSTANT_REQ			4

#define OP_CALIBRATION_RTD_CONSTANT_SET		1
#define OP_CALIBRATION_NTC_CON_TABLE_CAL	2
#define OP_CALIBRATION_NTC_CONSTANT_SET		3
#define OP_CALIBRATION_RTD_CONSTANT_REQ		4
#define OP_CALIBRATION_NTC_CON_TABLE_REQ	5
#define OP_CALIBRATION_NTC_CONSTANT_REQ		6

#define OP_SDCARD_LIST          			1
#define OP_SDCARD_LIST_START       			1
#define OP_SDCARD_LIST_BODY        			2
#define OP_SDCARD_LIST_END        			3
#define OP_SDCARD_DOWNLOAD		  			4
#define OP_SDCARD_DOWNLOAD_HEADER  			4
#define OP_SDCARD_DOWNLOAD_BADY    			5
#define OP_SDCARD_DOWNLOAD_FOOTER 			6
#define OP_SDCARD_DELETE        			7
#define OP_SDCARD_FORMAT        			8
#define OP_SDCARD_ERROR         			9

#define OP_SLOT_SET             			1
#define OP_SLOT_REQ             			2

#define OP_TIME_SET             			1
#define OP_TIME_REQ             			2


//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
//uint32_t  RecieveBTUart(uint8_t *key);
//void SendBTUart(uint8_t c);
void SendUart485String(uint8_t *data, uint16_t length);
void SendUart485NonDma(uint8_t *data, uint16_t length);
void StartRs485Task(void const * argument);
void DoReadFileList(void);

void doSaveIntervalTime(void);

void RxFunction(void);
void UnpackingRs485RxQueue(void);

//void ParsingFromRs485(void);
void JumpToFunction485(void);
void DoSendFile(void);	
void DoSendFileOpen(void);
void DoSendFileBodyPacket(uint32_t Offset, UINT packetSize);
void DoSendFileClose(void);
int32_t DoSendFileRead(FSIZE_t Offset, UINT ReadSize);

void CmdWarningTempSet(void);
void CmdWarningTempReq(void);
void CmdCalibrationRTDConstSet(void);
void CmdCalibrationNTCTableCal(void);
void CmdCalibrationNTCConstantSet(void);
void CmdCalibrationRTDConstReq(void);
void CmdCalibrationNTCTableReq(void);
void CmdCalibrationNTCConstantReq(void);
void CmdRevisionApplySet(void);
void CmdRevisionConstantSet(void);
void CmdRevisionApplyReq(void);
void CmdRevisionConstantReq(void);

void doSetTime(void);
void doGetTime(void);


#endif

