#include "frame.h"
#include "stm32f4xx_hal.h"
#include "0_Util.h"
#include "debug.h"
#include <string.h>
#include <stdlib.h>

/* CRC16 implementation acording to CCITT standards */

static const uint16_t crc16tab[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
	0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
	0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
	0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
	0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
	0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
	0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
	0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
	0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
	0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
	0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
	0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
	0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
	0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
	0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
	0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
	0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
	0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
	0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
	0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
	0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
	0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
	0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
	0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
	0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
	0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

__STATIC_INLINE uint16_t crc16_ccitt_update(uint16_t crc, uint8_t a)
{
	return (crc << 8) ^ crc16tab[((crc >> 8) ^ a) & 0x00FF];
}

static uint16_t crc16_ccitt(const uint8_t *buf, int len)
{
	register int counter;
	register uint16_t crc = 0;
	for (counter = 0; counter < len; counter++)
		/* crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *(uint8_t *)buf++) & 0x00FF]; */
		crc = crc16_ccitt_update(crc, *(uint8_t *)buf++);
	return crc;
}

__STATIC_INLINE uint16_t crc16_modbus_update(uint16_t crc, uint8_t a)
{
	register int i;
	crc ^= (uint16_t)a;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}

	return crc;
}
static uint16_t crc16_modbus(const uint8_t *buf, int len)
{
    register uint16_t crc = 0xFFFF;
    register int i;
    for (i = 0; i < len; i++) {
        crc = crc16_modbus_update(crc, *(uint8_t *)buf++);
    }
    return crc;
}

int fill_internal_frame(uint8_t *buffer, uint8_t slot_id, uint8_t cmd,
                        uint8_t datalen, uint8_t *data)
{
	if (!buffer)
		return 0;

	uint8_t *offset = buffer;

	*offset++ = INT_STX; /* starting preamble */
	*offset++ = slot_id;
	*offset++ = cmd;
	*offset++ = datalen;

	if (datalen > 0) {
		memcpy(offset, data, datalen);
		offset += datalen;
	}

	*((uint16_t *)offset) = crc16_ccitt(buffer + 1, 3 + datalen);
	offset += 2;

	*offset = INT_ETX;

	return 7 + datalen;
}

int parse_internal_frame(struct internal_frame *frm, uint8_t const *byte)
{
	static uint8_t state = 0;
	static uint8_t datalen = 0;
	static uint8_t received_crc[2] = {0};
	static uint16_t crc = 0;

	if (!frm || !byte)
		return 0;

	switch (state) {
	case 0: { /* starting preamble */
		if (*byte == INT_STX) {
			datalen = 0;
			received_crc[0] = 0;
			received_crc[1] = 0;
			crc = 0;
			++state;
		}
		break;
	}

	case 1: { /* slot_id */
		frm->slot_id = *byte;
		crc = crc16_ccitt_update(crc, *byte);
		++state;
		break;
	}

	case 2: { /* cmd */
		frm->cmd = *byte;
		crc = crc16_ccitt_update(crc, *byte);
		++state;
		break;
	}

	case 3: { /* datalen */
		frm->datalen = *byte;
		crc = crc16_ccitt_update(crc, *byte);
		++state;
		if (frm->datalen == 0)
			++state;
		else if (frm->datalen > 0xFF)
			state = 0;
		else {
			datalen = frm->datalen;
			memset(frm->data, 0, datalen);
		}
		break;
	}

	case 4: { /* data */
		frm->data[frm->datalen - datalen] = *byte;
		crc = crc16_ccitt_update(crc, *byte);
		if (--datalen == 0) {
			++state;
		}
		break;
	}

	case 5: { /* crc msb */
		received_crc[0] = *byte;
		++state;
		break;
	}

	case 6: { /* crc lsb */
		received_crc[1] = *byte;
		if (*((uint16_t *)received_crc) != crc) {
			state = 0;
			return 0;
		}
		++state;
		break;
	}

	case 7: {
		if (*byte != INT_ETX) {
			state = 0;
			return 0;
		}
		state = 0;
		return 1;
	}
	}

	return 0;
}

int fill_external_tx_frame(uint8_t *buffer, uint8_t cmd, uint8_t option,
			   uint8_t* data, uint32_t datalen)
{
	if (!buffer)
		return 0;

	uint8_t *offset = buffer;

	*offset++ = EXT_STX;
	*offset++ = cmd;
	*offset++ = option;

	/* packet total length */
	*((uint32_t *) offset) = datalen + 20;
	offset += sizeof(uint32_t);

	/* ipaddr */
	*offset++ = 192;
	*offset++ = 168;
	*offset++ = 255;
	*offset++ = 255;

	/* date & time */
	*offset++ = SysTime.Date.Year;
	*offset++ = SysTime.Date.Month;
	*offset++ = SysTime.Date.Date;
	*offset++ = SysTime.Time.Hours;
	*offset++ = SysTime.Time.Minutes;
	*offset++ = SysTime.Time.Seconds;

	if (datalen > 0) {
		memcpy(offset, data, datalen);
		offset += datalen;
	}

	*((uint16_t *)offset) = crc16_modbus(buffer + 1, datalen + 20 - 4);
	offset += 2;

	*offset = EXT_ETX;

	return datalen + 20;
}

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
extern uint8_t ext_rx_buffer[128];

int parse_external_rx_frame(struct external_frame_rx *frm, uint8_t const *byte)
{
	static uint8_t state = 0;
	static uint8_t datalen = 0;
	static uint8_t ipaddrlen = 0;
	static uint8_t received_crc[2] = {0};
	static uint16_t crc = 0xFFFF;

	if (!frm)
		return 0;

	switch (state) {
	case 0: {
		if (*byte == EXT_STX) {
			received_crc[0] = 0;
			received_crc[1] = 0;
			crc = 0xFFFF;
			++state;
		}
		break;
	}

	case 1: {		/* cmd */
		frm->cmd = *byte;
		crc = crc16_modbus_update(crc, *byte);
		++state;
		break;
	}

	case 2: {		/* option */
		frm->option = *byte;
		crc = crc16_modbus_update(crc, *byte);
		ipaddrlen = 4;
		++state;
		break;
	}

	case 3: {		/* ipaddr */
		frm->ipaddr[4 - ipaddrlen] = *byte;
		crc = crc16_modbus_update(crc, *byte);
		if (--ipaddrlen == 0) {
			datalen = 22;
			++state;
		}
		break;
	}

	case 4: {		/* data */
		frm->data[EXT_FRAME_RX_DATA_SIZE - datalen] = *byte;
		crc = crc16_modbus_update(crc, *byte);
		if (--datalen == 0) {
			++state;
		}
		break;
	}

	case 5: {
		received_crc[0] = *byte;
		++state;
		break;
	}

	case 6: {
		received_crc[1] = *byte;
		if (crc != *((uint16_t*)received_crc)) {
			DBG_LOG("%s: mismatch with crc, received (0x%04X), calcurated (0x%04X)\n", __func__,
				*((uint16_t*)received_crc), crc);
			state = 0;
			break;
		}
		++state;
		break;
	}

	case 7: { /* ending preamble */
		/* HAL_UART_Receive_DMA(&huart1, ext_rx_buffer, ARRAY_LEN(ext_rx_buffer)); */
		state = 0;
		return (*byte == EXT_ETX) ? 1 : 0;
	}
	}

	return 0;
}
