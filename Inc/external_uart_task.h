#ifndef EXTERNAL_UART_TASK_H
#define EXTERNAL_UART_TASK_H

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "0_Util.h"
#include "0_BCD110DS.h"

/* #include "stm32f4xx_hal_conf.h" */

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

#define RS_READ_SIZE (32)

#define RS_STX (0x7F)
#define RS_ETX (0x7E)

#define COMMAND (5)
#define OPTION (6)
#define DATA (7)

#define CMD_TEMP_TEST 1
#define CMD_WARNING 2
#define CMD_COMPENSATE 3
#define CMD_CORRECTION 4
#define CMD_SD_CARD 5
#define CMD_SLOT 6
#define CMD_TIME 7
#define CMD_SYS 0xF

#define OP_TEMP_START_RX 1
#define OP_TEMP_MAIN_INFO 1
#define OP_TEMP_SLOT_INFO 2
#define OP_TEMP_CHANNEL_INFO 3
#define OP_TEMP_CHANNEL_VALUE 4
#define OP_TEMP_STOP 5
#define OP_TEMP_SAMPLE_RATE 6

#define OP_WARNING_THRESHOLD_SET 1
#define OP_WARNING_THRESHOLD_REQ 2
#define OP_WARNING_VARIATION_SET 3
#define OP_WARNING_VARIATION_REQ 4

#define OP_COMPENSATED_APPLY_SET 1
#define OP_COMPENSATED_CONSTANT_SET 2
#define OP_COMPENSATED_APPLY_REQ 3
#define OP_COMPENSATED_CONSTANT_REQ 4
/* Thermal Resistance Constant */
#define OP_COMPENSATED_TR_CONST_SET 5
#define OP_COMPENSATED_TR_CONST_REQ 6

#define OP_CORRECTION_RTD_CONSTANT_SET 1
#define OP_CORRECTION_NTC_CON_TABLE_CAL 2
#define OP_CORRECTION_NTC_CONSTANT_SET 3
#define OP_CORRECTION_RTD_CONSTANT_REQ 4
#define OP_CORRECTION_NTC_CON_TABLE_REQ 5
#define OP_CORRECTION_NTC_CONSTANT_REQ 6

#define OP_SDCARD_LIST 1
#define OP_SDCARD_LIST_START 1
#define OP_SDCARD_LIST_BODY 2
#define OP_SDCARD_LIST_END 3
#define OP_SDCARD_DOWNLOAD 4
#define OP_SDCARD_DOWNLOAD_HEADER 0xF4
#define OP_SDCARD_DOWNLOAD_START 4
#define OP_SDCARD_DOWNLOAD_BODY 5
#define OP_SDCARD_DOWNLOAD_END 6
#define OP_SDCARD_DELETE 7
#define OP_SDCARD_FORMAT 8
#define OP_SDCARD_ERROR 9

#define OP_SLOT_SET 1
#define OP_SLOT_REQ 2

#define OP_TIME_SET 1
#define OP_TIME_REQ 2

#define OP_FIRMWARE_VERSION_REQ 1
#define OP_STARTED_NOTIFY 2

struct ext_rx_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

void ext_rx_notify();
void receive_from_external(struct external_frame_rx *);
int send_to_external(uint8_t cmd, uint8_t option, void *data,
		uint16_t data_writhe_len, uint16_t data_len, uint16_t buffer_len);

void external_rx_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* EXTERNAL_UART_TASK_H */

