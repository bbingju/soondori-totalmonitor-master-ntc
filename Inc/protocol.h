#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "cmsis_os.h"
#include "cmsis_gcc.h"
#include "frame.h"

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
#define OP_SDCARD_DOWNLOAD		  		4
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

#define ALL_SLOT            0xFF


struct internal_rx_msg_s {
    uint8_t id;
    uint8_t cmd;
    size_t datalen;
    uint8_t data[164];
};

__PACKED_STRUCT internal_tx_msg_s {
    uint8_t id;
    uint8_t cmd;
    uint8_t data[8];
    uint8_t length;
    int rx_dma_req_bytes;
};

#ifdef __cplusplus
extern "C" {
#endif

const char* ext_cmd_str(uint8_t cmd);
const char *ext_option_str(uint8_t cmd, uint8_t option);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
