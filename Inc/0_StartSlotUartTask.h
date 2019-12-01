#ifndef __START_SLAVE_TX_TASK_H__
#define __START_SLAVE_TX_TASK_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "0_Util.h"

#ifndef __STM32F4xx_HAL_CONF_H
  #include "stm32f4xx_hal_conf.h"
#endif

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"




#define ALL_SLOT            0xF0

// to Slave UART
#define SEND_DATA_LENGTH					(uint8_t)(14)
#define CMD_BASE                			CMD_STX, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, CMD_ETX
#define CMD_STX     		    			(uint8_t)0x02
#define CMD_ETX     		    			(uint8_t)0x03
#define CMD_BOARD_TYPE		    			(uint8_t)'a'		//Board Type ��û
#define CMD_BOARD_EN_REQ					(uint8_t)'b'		//Board ��� ��û
#define CMD_BOARD_EN_SET					(uint8_t)'c'		//Board ��� ����
#define CMD_BOARD_RESET		    			(uint8_t)'d'		//Board Reset
#define CMD_SLOT_ID_REQ						(uint8_t)'e'		//Slot ��û
#define CMD_HW_VER							(uint8_t)'f'		//HW Version ��û
#define CMD_FW_VER							(uint8_t)'g'		//FW Version ��û
#define CMD_UUID_REQ 						(uint8_t)'h'		//UUID ��û
#define CMD_TEMP_REQ 						(uint8_t)'i'		//Temp ��û
#define CMD_TEMP_STATE_REQ 					(uint8_t)'j'		//Temp ���� ��û
#define CMD_ADC_REQ							(uint8_t)'k'		//ADC ��û
#define CMD_THRESHOLD_REQ	    			(uint8_t)'l'		//���µ� ��û
#define CMD_THRESHOLD_SET	    			(uint8_t)'m'		//���µ� ����
#define CMD_RELAY_REQ 		    			(uint8_t)'n'		//RELAY ���� ��û
#define CMD_RELAY_SET 		    			(uint8_t)'o'		//RELAY ��� ����

#define CMD_REVISION_APPLY_SET     			(uint8_t)'p'		//���� ����
#define CMD_REVISION_CONSTANT_SET			(uint8_t)'q'		//���˻�� ����
#define CMD_REVISION_APPLY_REQ      		(uint8_t)'r'		//�������� ��û
#define CMD_REVISION_CONSTANT_REQ			(uint8_t)'s'		//���˻�� ��û

#define CMD_CALIBRATION_NTC_CON_TABLE_CAL	(uint8_t)'t'		//NTC������� ����ض�
#define CMD_CALIBRATION_NTC_CONSTANT_SET	(uint8_t)'u'		//NTC������� ���� ����
#define CMD_CALIBRATION_NTC_CON_TABLE_REQ	(uint8_t)'v'		//NTC������� ��û
#define CMD_CALIBRATION_NTC_CONSTANT_REQ	(uint8_t)'w'		//NTC�������� ��û

#define CMD_REQ         	    			(uint8_t)0x01
#define CMD_SET         	    			(uint8_t)0x02

//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartSlotUartTask(void const * argument);

//  ����
void SlotRxFunction(void);
void UnpackingRxQueue(void);
void DoSlotReset(uint8_t slot);
void DoSelectSlot(uint8_t slot);
void DoRejectSlot(void);

//  Tx
void UartInternalTxFunction(uint8_t* datas, uint16_t length);
void DoCalibrationNTCTableCal(uint8_t slotNumber);
void DoCalibrationNTCConstantSet(uint8_t slotNumber);
void DoCalibrationNTCTableReq(uint8_t slotNumber);
void DoCalibrationNTCConstantReq(uint8_t slotNumber);
void DoThresholdSet(uint8_t slotNumber, uint8_t channal, uni4Byte thresholdTemp);
void DoThresholdReq(uint8_t slotNumber);
void DoReqTemperature(uint8_t slotNumber);
void DoReqTeameratureState(uint8_t slotNumber);
void DoReqSlotID(uint8_t slotNumber);
void DoRevisionApplySet(uint8_t slotNumber, uint8_t mode);
void DoRevisionConstantSet(uint8_t slotNumber);
void DoRevisionApplyReq(uint8_t slotNumber);
void DoRevisionConstantReq(uint8_t slotNumber);

void DoIncSlotIdStep(uint8_t slotNumber);

//  Rx
void DoSlotJumpFunction(void);
void DoAnsBoardType(void); 
void DoAnsReqSlotID(void);
void DoAnsTemperature(void);
void DoAnsTemperatureState(void);
void DoAnsThresholdReq(void);
void DoAnsThresholdSet(void);
void DoAnsCalibrationNTCTableCal(void);
void DoAnsCalibrationNTCTableReq(void);
void DoAnsCalibrationNTCConstantSet(void);
void DoAnsCalibrationNTCConstantReq(void);
void DoAnsRevisionApplySet(void);
void DoAnsRevisionConstantSet(void);
void DoAnsRevisionApplyReq(void);
void DoAnsRevisionConstantReq(void);


#endif

