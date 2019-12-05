#include "cmsis_os.h"
#include "0_Util.h"
#include "internal_rx_task.h"
#include <string.h>
#include "0_StartSlotUartTask.h"

struct internal_rx_msg_s {
    uint8_t id;
    uint8_t type;
    uint16_t length;
    uint8_t rawdata[134];
    uint8_t *data;
};

osMailQDef(internal_rx_pool_q, 8, struct internal_rx_msg_s);
osMailQId (internal_rx_pool_q_id);

static int validate_msg(uint8_t *rawdata, uint16_t length)
{

    if (rawdata[0] == CMD_STX && rawdata[length - 1] == CMD_ETX)
        return 0;

    return -1;
}

int internal_rx_msg_push(void *data, uint16_t length)
{
    if (length > 134 || data == NULL) {
        return -1;
    }

    if (validate_msg(data, length) < 0) {
        DBG_LOG("%s: error validate_msg\n", __func__);
        print_bytes(data, length);
        return -1;
    }

    struct internal_rx_msg_s *obj;
    obj = (struct internal_rx_msg_s *) osMailAlloc(internal_rx_pool_q_id, osWaitForever);
    if (!obj) {
        return -1;
    }

    memcpy(obj->rawdata, data, length);
    obj->id = obj->rawdata[1];
    obj->type = obj->rawdata[2];
    obj->length = length - 4;
    obj->data = &obj->rawdata[3];

    osMailPut(internal_rx_pool_q_id, obj);

    return 0;
}

static const char* cmd_str(uint8_t cmd)
{
    switch (cmd) {
    case CMD_BOARD_TYPE:
        return "CMD_BOARD_TYPE";
    case CMD_BOARD_EN_REQ:
        return "CMD_BOARD_EN_REQ";
    case CMD_BOARD_EN_SET:
        return "CMD_BOARD_EN_SET";
    case CMD_SLOT_ID_REQ:
        return "CMD_SLOT_ID_REQ";
    case CMD_HW_VER:
        return "CMD_HW_VER";
    case CMD_FW_VER:
        return "CMD_FW_VER";
    case CMD_UUID_REQ:
        return "CMD_UUID_REQ";
    case CMD_ADC_REQ:
        return "CMD_ADC_REQ";
    case CMD_RELAY_REQ:
        return "CMD_RELAY_REQ";
    case CMD_RELAY_SET:
        return "CMD_RELAY_SET";
    case CMD_REVISION_APPLY_SET:
        return "CMD_REVISION_APPLY_SET";
    case CMD_REVISION_CONSTANT_SET:
        return "CMD_REVISION_CONSTANT_SET";
    case CMD_REVISION_APPLY_REQ:
        return "CMD_REVISION_APPLY_REQ";
    case CMD_REVISION_CONSTANT_REQ:
        return "CMD_REVISION_CONSTANT_REQ";
    case CMD_CALIBRATION_NTC_CONSTANT_SET:
        return "CMD_CALIBRATION_NTC_CONSTANT_SET";
    case CMD_CALIBRATION_NTC_CONSTANT_REQ:
        return "CMD_CALIBRATION_NTC_CONSTANT_REQ";
    case CMD_TEMP_STATE_REQ:
        return "CMD_TEMP_STATE_REQ";
    case CMD_TEMP_REQ:
        return "CMD_TEMP_REQ";
    case CMD_THRESHOLD_REQ:
        return "CMD_THRESHOLD_REQ";
    case CMD_THRESHOLD_SET:
        return "CMD_THRESHOLD_SET";
    case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
        return "CMD_CALIBRATION_NTC_CON_TABLE_CAL";
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        return "CMD_CALIBRATION_NTC_CON_TABLE_REQ";
    default:
        return "";
    }
}

static void handle_msg(struct internal_rx_msg_s *received)
{
    if (received->type == CMD_THRESHOLD_SET || received->type == CMD_THRESHOLD_REQ) {
    /* print_bytes(received->rawdata, received->length + 4); */
    DBG_LOG("[%d] %s: (%d) ",
            received->id - 0x30, cmd_str(received->type), received->length);
    print_bytes(received->data, received->length);
    }

    switch (received->type) {
    case CMD_BOARD_TYPE:
        break;
    case CMD_BOARD_EN_REQ:
        break;
    case CMD_BOARD_EN_SET:
        break;
    case CMD_SLOT_ID_REQ:
        break;
    case CMD_HW_VER:
        break;
    case CMD_FW_VER:
        break;
    case CMD_UUID_REQ:
        break;
    case CMD_ADC_REQ:
        break;
    case CMD_RELAY_REQ:
        break;
    case CMD_RELAY_SET:
        break;
    case CMD_REVISION_APPLY_SET:
        break;
    case CMD_REVISION_CONSTANT_SET:
        break;
    case CMD_REVISION_APPLY_REQ:
        break;
    case CMD_REVISION_CONSTANT_REQ:
        break;
    case CMD_CALIBRATION_NTC_CONSTANT_SET:
        break;
    case CMD_CALIBRATION_NTC_CONSTANT_REQ:
        break;
    case CMD_TEMP_STATE_REQ:
        break;
    case CMD_TEMP_REQ:
        break;
    case CMD_THRESHOLD_REQ:
        break;
    case CMD_THRESHOLD_SET:
        break;
    case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
        break;
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        break;
    }
}

void internal_rx_task(void const *arg)
{
    internal_rx_pool_q_id = osMailCreate(osMailQ(internal_rx_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(internal_rx_pool_q_id, osWaitForever);
        struct internal_rx_msg_s *received = (struct internal_rx_msg_s *) event.value.p;
        handle_msg(received);
        osMailFree(internal_rx_pool_q_id, received);
    }
}
