#include "protocol.h"

const char *ext_cmd_str(uint8_t cmd)
{
    switch (cmd) {
    case CMD_TEMP_TEST:
        return "CMD_TEMP_TEST";
    case CMD_WARNING_TEMP:
        return "CMD_WARNING_TEMP";
    case CMD_REVISION:
        return "CMD_REVISION";
    case CMD_CALIBRATION:
        return "CMD_CALIBRATION";
    case CMD_SD_CARD:
        return "CMD_SD_CARD";
    case CMD_SLOT:
        return "CMD_SLOT";
    case CMD_TIME:
        return "CMD_TIME";
    }

    return "";
}

const char *ext_option_str(uint8_t cmd, uint8_t option)
{
	switch (cmd) {
	case CMD_TEMP_TEST:
		switch (option) {
		case OP_TEMP_START_RX:
			return "OP_TEMP_START_RX";
		case OP_TEMP_STOP:
			return "OP_TEMP_STOP";
		case OP_TEMP_SAMPLE_RATE:
			return "OP_TEMP_SAMPLE_RATE";
		case OP_TEMP_SLOT_INFO:
			return "OP_TEMP_SLOT_INFO";
		case OP_TEMP_CHANNEL_INFO:
			return "OP_TEMP_CHANNEL_INFO";
		case OP_TEMP_CHANNEL_VALUE:
			return "OP_TEMP_CHANNEL_VALUE";
		}
		break;
	case CMD_WARNING_TEMP:
		switch (option) {
		case OP_WARNING_TEMP_SET:
			return "OP_WARNING_TEMP_SET";
		case OP_WARNING_TEMP_REQ:
			return "OP_WARNING_TEMP_REQ";
		}
		break;
	case CMD_REVISION:
		switch (option) {
		case OP_REVISION_APPLY_SET:
			return "OP_REVISION_APPLY_SET";
		case OP_REVISION_CONSTANT_SET:
			return "OP_REVISION_CONSTANT_SET";
		case OP_REVISION_APPLY_REQ:
			return "OP_REVISION_APPLY_REQ";
		case OP_REVISION_CONSTANT_REQ:
			return "OP_REVISION_CONSTANT_REQ";
		}
		break;
	case CMD_CALIBRATION:
		switch (option) {
		case OP_CALIBRATION_RTD_CONSTANT_SET:
			return "OP_CALIBRATION_RTD_CONSTANT_SET";
		case OP_CALIBRATION_NTC_CON_TABLE_CAL:
			return "OP_CALIBRATION_NTC_CON_TABLE_CAL";
		case OP_CALIBRATION_NTC_CONSTANT_SET:
			return "OP_CALIBRATION_NTC_CONSTANT_SET";
		case OP_CALIBRATION_RTD_CONSTANT_REQ:
			return "OP_CALIBRATION_RTD_CONSTANT_REQ";
		case OP_CALIBRATION_NTC_CON_TABLE_REQ:
			return "OP_CALIBRATION_NTC_CON_TABLE_REQ";
		case OP_CALIBRATION_NTC_CONSTANT_REQ:
			return "OP_CALIBRATION_NTC_CONSTANT_REQ";
		}
		break;
	case CMD_SD_CARD:
		switch (option) {
		case OP_SDCARD_LIST_START:
			return "OP_SDCARD_LIST_START";
		case OP_SDCARD_LIST_BODY:
			return "OP_SDCARD_LIST_BODY";
		case OP_SDCARD_LIST_END:
			return "OP_SDCARD_LIST_END";
			/* case OP_SDCARD_LIST: */
			/*     return "OP_SDCARD_LIST"; */
			/* case OP_SDCARD_DOWNLOAD: */
			/*     return "OP_SDCARD_DOWNLOAD"; */
		case OP_SDCARD_DOWNLOAD_HEADER:
			return "OP_SDCARD_DOWNLOAD_HEADER";
		case OP_SDCARD_DOWNLOAD_BADY:
			return "OP_SDCARD_DOWNLOAD_BADY";
		case OP_SDCARD_DOWNLOAD_FOOTER:
			return "OP_SDCARD_DOWNLOAD_FOOTER";
		case OP_SDCARD_DELETE:
			return "OP_SDCARD_DELETE";
		case OP_SDCARD_FORMAT:
			return "OP_SDCARD_FORMAT";
		case OP_SDCARD_ERROR:
			return "OP_SDCARD_ERROR";
		}
		break;
	case CMD_SLOT:
		switch (option) {
		case OP_SLOT_SET:
			return "OP_SLOT_SET";
		case OP_SLOT_REQ:
			return "OP_SLOT_REQ";
		}
		break;
	case CMD_TIME:
		switch (option) {
		case OP_TIME_SET:
			return "OP_TIME_SET";
		case OP_TIME_REQ:
			return "OP_TIME_REQ";
		}
		break;
	}
	return "";
}
