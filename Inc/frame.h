#ifndef _FRAME_H
#define _FRAME_H

#include <stdint.h>
#include "cmsis_compiler.h"

/**
 * Commands for Internal Communication
 */
#define INTERNAL_CMD_BOARD_TYPE_REQ 0x01
#define INTERNAL_CMD_RESET 0x02
#define INTERNAL_CMD_SLOT_ID_REQ 0x03
#define INTERNAL_CMD_TEMPERATURE_REQ 0x04
#define INTERNAL_CMD_TEMPERATURE_STATE_REQ 0x05
#define INTERNAL_CMD_ADC_REQ 0x06
#define INTERNAL_CMD_THRESHOLD_REQ 0x07
#define INTERNAL_CMD_THRESHOLD_SET 0x08
#define INTERNAL_CMD_RELAY_REQ 0x09
#define INTERNAL_CMD_RELAY_SET 0x0A
#define INTERNAL_CMD_REVISION_CONSTANT_REQ 0x0B
#define INTERNAL_CMD_REVISION_CONSTANT_SET 0x0C
#define INTERNAL_CMD_REVISION_APPLY_REQ 0x0D
#define INTERNAL_CMD_REVISION_APPLY_SET 0x0E
#define INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_CAL 0x0F
#define INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_REQ 0x10
#define INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_REQ 0x11
#define INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_SET 0x12

#define INT_STX 0xFE
#define INT_ETX 0xFD
#define EXT_STX 0x7F
#define EXT_ETX 0x7E

#define TEMP_STATE_DATA_LENGTH 16

__PACKED_STRUCT internal_temp_data {
	float values[32];
};

__PACKED_STRUCT internal_temp_state_data {
	uint8_t states[TEMP_STATE_DATA_LENGTH];
};

struct internal_frame {
	uint8_t slot_id;
	uint8_t cmd;
	uint8_t datalen;
	union {
		struct internal_temp_data temp_data;
		struct internal_temp_state_data state_data;
		uint8_t data[256];
	};
};

__PACKED_STRUCT external_frame_rx {
	uint8_t cmd;
	uint8_t option;
	uint8_t ipaddr[4];
	union {
		__PACKED_STRUCT {
			uint8_t slot_id;
			uint8_t channel;
			float value;
		} warning_temp_set;

		__PACKED_STRUCT {
			uint32_t sec;
		} interval_set;

		uint8_t data[22];
	};
};

__PACKED_STRUCT external_temp_data {
	uint32_t slot_id;
	float values[32];
	uint8_t padding[3];
};

__PACKED_STRUCT external_temp_state_data {
	uint8_t slot_id;
	uint8_t states[32];
	uint8_t padding[3];
};

struct external_frame_tx {
	uint8_t cmd;
	uint8_t option;
	/* STX, ETX까지 포함된 길이. 즉, data length + 20. */
	uint32_t len;
	uint8_t ipaddr[4];
	uint8_t datetime[6];
	uint8_t data_padding_len;
	union {
		struct external_temp_data temp_data;
		struct external_temp_state_data temp_state_data;
		uint8_t data[256];
	};
};

#ifdef __cplusplus
extern "C" {
#endif

int fill_internal_frame(uint8_t *buffer, uint8_t slot_id, uint8_t cmd, uint8_t datalen, uint8_t *data);
int parse_internal_frame(struct internal_frame *frm, uint8_t const *byte);

int fill_external_tx_frame(uint8_t *buffer, uint8_t cmd, uint8_t option,
			uint8_t *ipaddr, uint8_t *datetime, uint8_t* data, uint32_t datalen);
int parse_external_rx_frame(struct external_frame_rx *frm, uint8_t const *byte);

#ifdef __cplusplus
}
#endif

#endif /* _FRAME_H */
