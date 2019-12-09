#include "cmsis_os.h"
#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "internal_tx_task.h"
#include <string.h>


extern uint8_t SendSlotNumber;
extern uint8_t noReturnSendCt;

struct internal_tx_msg_s {
    uint8_t id;
    uint8_t cmd;
    uint8_t data[6];
    uint16_t length;
    int rx_dma_req_bytes;
} __packed;

osMailQDef(internal_tx_pool_q, 8, struct internal_tx_msg_s);
osMailQId (internal_tx_pool_q_id);

struct internal_tx_msg_s *tx_received;

static int bytes_to_request(uint8_t cmd)
{
    switch (cmd) {
    case CMD_BOARD_TYPE:
    case CMD_BOARD_EN_REQ:
    case CMD_BOARD_EN_SET:
    case CMD_HW_VER:
    case CMD_FW_VER:
    case CMD_UUID_REQ:
    case CMD_ADC_REQ:
    case CMD_RELAY_REQ:
    case CMD_RELAY_SET:
        return -1;

    case CMD_SLOT_ID_REQ:
        return 12;

    case CMD_REVISION_APPLY_SET:
    case CMD_REVISION_CONSTANT_SET:
    case CMD_REVISION_APPLY_REQ:
    case CMD_REVISION_CONSTANT_REQ:
    case CMD_CALIBRATION_NTC_CONSTANT_SET:
    case CMD_CALIBRATION_NTC_CONSTANT_REQ:
        return 32;

    case CMD_TEMP_STATE_REQ:
        return 38;

    case CMD_TEMP_REQ:
    case CMD_THRESHOLD_REQ:
    case CMD_THRESHOLD_SET:
        return 134;

    case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        return 152;
    }
    return -1;
}

int send_internal_msg(uint8_t id, uint8_t cmd, void *data, uint16_t length)
{
    if (length > 134 || id > 3) {
        return -1;
    }

    struct internal_tx_msg_s *obj;
    obj = (struct internal_tx_msg_s *) osMailAlloc(internal_tx_pool_q_id, osWaitForever);
    if (!obj) {
        return -1;
    }

    memset(obj->data, 0, sizeof(obj->data));
    if (data)
        memcpy(obj->data, data, length);
    obj->id = id;
    obj->cmd = cmd;
    obj->length = length;
    obj->rx_dma_req_bytes = bytes_to_request(obj->cmd);

    osMailPut(internal_tx_pool_q_id, obj);
    return 0;
}

static uint8_t recv_buffer[256];

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

static void handle_tx_msg(struct internal_tx_msg_s *received)
{
    static uint8_t buf[SEND_DATA_LENGTH] = { 0 };

    memset(buf, 00, sizeof(buf));
    doMakeSendSlotData(buf, received->id + 0x30, received->cmd,
                       received->data, received->length, SEND_DATA_LENGTH);
    noReturnSendCt++;

    if (received->cmd == CMD_THRESHOLD_SET || received->cmd == CMD_THRESHOLD_REQ) {
        DBG_LOG("int tx [%d] %s: (%d) ",
                received->id, cmd_str(received->cmd), received->length);
        print_bytes(received->data, received->length);
    }

    HAL_UART_Transmit_DMA(&huart2, buf, SEND_DATA_LENGTH);
    /* osDelay(1); */

    if (received->rx_dma_req_bytes != -1) {
        HAL_UART_Receive_DMA(&huart2, recv_buffer, received->rx_dma_req_bytes);
    }
}

void internal_tx_task(void const *arg)
{
    internal_tx_pool_q_id = osMailCreate(osMailQ(internal_tx_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(internal_tx_pool_q_id, osWaitForever);
        tx_received = (struct internal_tx_msg_s *) event.value.p;
        handle_tx_msg(tx_received);
        if (tx_received->rx_dma_req_bytes == -1) {
            osMailFree(internal_tx_pool_q_id, tx_received);
            tx_received = NULL;
        }
    }
}

void send_slot_id_req(uint8_t id)
{
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
    DoRejectSlot();

    uint8_t internal_id = id + 0x30;

    send_internal_msg(id, CMD_SLOT_ID_REQ, &internal_id, 1);
}
