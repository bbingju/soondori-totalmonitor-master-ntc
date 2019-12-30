#ifndef INTERNAL_UART_TASK_H
#define INTERNAL_UART_TASK_H

#include "0_GlobalValue.h"
#include <stdint.h>

// to Slave UART
#define SEND_DATA_LENGTH					((uint8_t)(14))
#define CMD_STX     		    			((uint8_t)0x02)
#define CMD_ETX     		    			((uint8_t)0x03)
#define CMD_BOARD_TYPE		    			((uint8_t)'a')		//Board Type 요청
#define CMD_BOARD_EN_REQ					((uint8_t)'b')		//Board 사용 요청
#define CMD_BOARD_EN_SET					((uint8_t)'c')		//Board 사용 설정
#define CMD_BOARD_RESET		    			((uint8_t)'d')		//Board Reset
#define CMD_SLOT_ID_REQ						((uint8_t)'e')		//Slot 요청
#define CMD_HW_VER							((uint8_t)'f')		//HW Version 요청
#define CMD_FW_VER							((uint8_t)'g')		//FW Version 요청
#define CMD_UUID_REQ 						((uint8_t)'h')		//UUID 요청
#define CMD_TEMP_REQ 						((uint8_t)'i')		//Temp 요청
#define CMD_TEMP_STATE_REQ 					((uint8_t)'j')		//Temp 상태 요청
#define CMD_ADC_REQ							((uint8_t)'k')		//ADC 요청
#define CMD_THRESHOLD_REQ	    			((uint8_t)'l')		//경고온도 요청
#define CMD_THRESHOLD_SET	    			((uint8_t)'m')		//경고온도 설정
#define CMD_RELAY_REQ 		    			((uint8_t)'n')		//RELAY 상태 요청
#define CMD_RELAY_SET 		    			((uint8_t)'o')		//RELAY 사용 설정

#define CMD_REVISION_APPLY_SET     			((uint8_t)'p')		//보정 적용
#define CMD_REVISION_CONSTANT_SET			((uint8_t)'q')		//접촉상수 설정
#define CMD_REVISION_APPLY_REQ      		((uint8_t)'r')		//보정상태 요청
#define CMD_REVISION_CONSTANT_REQ			((uint8_t)'s')		//접촉상수 요청

#define CMD_CALIBRATION_NTC_CON_TABLE_CAL	((uint8_t)'t')		//NTC교정상수 계산해라
#define CMD_CALIBRATION_NTC_CONSTANT_SET	((uint8_t)'u')		//NTC교정상수 증감 설정
#define CMD_CALIBRATION_NTC_CON_TABLE_REQ	((uint8_t)'v')		//NTC교정상수 요청
#define CMD_CALIBRATION_NTC_CONSTANT_REQ	((uint8_t)'w')		//NTC교정증감 요청

#define CMD_REQ         	    			((uint8_t)0x01)
#define CMD_SET         	    			((uint8_t)0x02)

struct internal_uart_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

void internal_rx_task(void const *arg);
void internal_rx_notify();
void send_slot_id_req(uint8_t id);
int send_internal_req(uint8_t id, uint8_t cmd, void *data, uint16_t length);
int push_internal_resp(void *data, uint16_t length);

void UartInternalTxFunction(uint8_t* datas, uint16_t length);
void DoCalibrationNTCTableCal(uint8_t slotNumber);
void DoCalibrationNTCConstantSet(uint8_t slotNumber);
void DoCalibrationNTCTableReq(uint8_t slotNumber);
void DoCalibrationNTCConstantReq(uint8_t slotNumber);
void DoThresholdSet(struct slot_properties_s *, uint8_t channal, float threshold);
void DoThresholdReq(struct slot_properties_s *);
void DoReqTemperature(struct slot_properties_s *);
void DoReqTemperatureState(struct slot_properties_s *);
void DoReqSlotID(uint8_t slotNumber);
void DoRevisionApplySet(uint8_t slotNumber, uint8_t const mode);
void DoRevisionConstantSet(uint8_t slotNumber);
void DoRevisionApplyReq(uint8_t slotNumber);
void DoRevisionConstantReq(uint8_t slotNumber);

void DoIncSlotIdStep(uint8_t slotNumber);

//  Rx
void DoSlotJumpFunction(void);
/* void DoAnsBoardType(void);  */
void DoAnsReqSlotID(void);
void DoAnsTemperature(void);
void DoAnsTemperatureState(void);
void DoAnsThresholdReq(void);
void DoAnsThresholdSet(void);
/* void DoAnsCalibrationNTCTableCal(void); */
/* void DoAnsCalibrationNTCTableReq(void); */
/* void DoAnsCalibrationNTCConstantSet(void); */
/* void DoAnsCalibrationNTCConstantReq(void); */
/* void DoAnsRevisionApplySet(void); */
/* void DoAnsRevisionConstantSet(void); */
/* void DoAnsRevisionApplyReq(void); */
/* void DoAnsRevisionConstantReq(void); */

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_UART_TASK_H */
