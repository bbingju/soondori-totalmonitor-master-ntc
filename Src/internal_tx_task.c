#include "cmsis_os.h"
#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "0_StartRs485Task.h"
#include "internal_tx_task.h"
#include <string.h>


extern uint8_t readSlotNumber;
extern uint8_t crcErrorCount;
extern uint8_t startThreshold;
extern uint8_t SendSlotNumber;
extern uint8_t noReturnSendCt;

struct internal_tx_msg_s {
    uint8_t id;
    uint8_t cmd;
    uint8_t data[6];
    uint16_t length;
    int rx_dma_req_bytes;
} __packed;

struct internal_rx_msg_s {
    uint8_t id;
    uint8_t type;
    uint16_t length;
    uint8_t rawdata[150];
    uint8_t *data;
};

struct internal_msg_s {
    struct internal_tx_msg_s request;
    struct internal_rx_msg_s response;
};

osMailQDef(internal_pool_q, 16, struct internal_msg_s);
osMailQId (internal_pool_q_id);

/* osMailQDef(internal_tx_pool_q, 1, struct internal_tx_msg_s); */
/* osMailQId (internal_tx_pool_q_id); */

/* struct internal_tx_msg_s *tx_received; */

static void handle_threshold_req(struct internal_rx_msg_s *msg);
static void handle_threshold_set(struct internal_rx_msg_s *msg);
static void handle_temperature(struct internal_rx_msg_s *msg);
static void handle_temerature_state(struct internal_rx_msg_s *msg);

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

int send_internal_req(uint8_t id, uint8_t cmd, void *data, uint16_t length)
{
    /* if (length > 134 || id > 3) { */
    /*     return -1; */
    /* } */

    struct internal_msg_s *obj;
    obj = (struct internal_msg_s *) osMailAlloc(internal_pool_q_id, osWaitForever);
    if (!obj) {
        DBG_LOG("%s: mail allocation failed\n", __func__);
        return -1;
    }

    struct internal_tx_msg_s *req = &obj->request;

    memset(req->data, 0, sizeof(req->data));
    if (data)
        memcpy(req->data, data, length);
    req->id = id;
    req->cmd = cmd;
    req->length = length;
    req->rx_dma_req_bytes = bytes_to_request(req->cmd);

    osMailPut(internal_pool_q_id, obj);
    return 0;
}

static int validate_msg(uint8_t *rawdata, uint16_t length)
{

    if (rawdata[0] == CMD_STX && rawdata[length - 1] == CMD_ETX)
        return 0;

    return -1;
}

/* int push_internal_resp(void *data, uint16_t length) */
/* { */
/*     if (length > 134 || data == NULL) { */
/*         return -1; */
/*     } */

/*     if (validate_msg(data, length) < 0) { */
/*         DBG_LOG("%s: error validate_msg\n", __func__); */
/*         print_bytes(data, length); */
/*         return -1; */
/*     } */

/*     memcpy(obj->rawdata, data, length); */
/*     obj->id = obj->rawdata[1] - '0'; */
/*     obj->type = obj->rawdata[2]; */
/*     obj->length = length - 4; */
/*     obj->data = &obj->rawdata[3]; */

/*     osMailPut(internal_rx_pool_q_id, obj); */

/*     return 0; */
/* } */

/* int send_internal_msg(uint8_t id, uint8_t cmd, void *data, uint16_t length) */
/* { */
/*     if (length > 134 || id > 3) { */
/*         return -1; */
/*     } */

/*     struct internal_tx_msg_s *obj; */
/*     obj = (struct internal_tx_msg_s *) osMailAlloc(internal_tx_pool_q_id, osWaitForever); */
/*     if (!obj) { */
/*         return -1; */
/*     } */

/*     memset(obj->data, 0, sizeof(obj->data)); */
/*     if (data) */
/*         memcpy(obj->data, data, length); */
/*     obj->id = id; */
/*     obj->cmd = cmd; */
/*     obj->length = length; */
/*     obj->rx_dma_req_bytes = bytes_to_request(obj->cmd); */

/*     osMailPut(internal_tx_pool_q_id, obj); */
/*     return 0; */
/* } */

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

    if (received->rx_dma_req_bytes != -1) {
        HAL_UART_Receive_DMA(&huart2, recv_buffer, received->rx_dma_req_bytes);
    }
}

static void handle_resp(struct internal_rx_msg_s *received)
{
    if (received->type == CMD_THRESHOLD_SET || received->type == CMD_THRESHOLD_REQ) {
    DBG_LOG("int rx [%d] %s: (%d) ",
            received->id, cmd_str(received->type), received->length);
    DBG_DUMP(received->data, received->length);
    }

    switch (received->type) {
    case CMD_BOARD_TYPE:
        break;
    case CMD_BOARD_EN_REQ:
        break;
    case CMD_BOARD_EN_SET:
        break;
    case CMD_SLOT_ID_REQ:
        noReturnSendCt = 0;
        if (SendSlotNumber == (received->id)) {
            SysProperties.slotInsert[received->id] = TRUE;
            DoIncSlotIdStep(SendSlotNumber);
        }
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
        handle_temerature_state(received);
        break;
    case CMD_TEMP_REQ:
        handle_temperature(received);
        break;
    case CMD_THRESHOLD_REQ:
        handle_threshold_req(received);
        break;
    case CMD_THRESHOLD_SET:
        handle_threshold_set(received);
        break;
    case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
        break;
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        break;
    }
}

int int_tx_completed;
int int_rx_completed;

static void handle_req(struct internal_msg_s *obj)
{
    static uint8_t buf[256] = { 0 };
    struct internal_tx_msg_s *req = &obj->request;
    struct internal_rx_msg_s *resp = &obj->response;

    memset(buf, 0, sizeof(buf));

    doMakeSendSlotData(buf, req->id + 0x30, req->cmd,
                       req->data, req->length, SEND_DATA_LENGTH);
    noReturnSendCt++;

    if (req->cmd == CMD_THRESHOLD_SET || req->cmd == CMD_THRESHOLD_REQ) {
        DBG_LOG("int tx [%d] %s: (%d) ",
                req->id, cmd_str(req->cmd), req->length);
        DBG_DUMP(req->data, req->length);
    }

    int_tx_completed = 0;
    HAL_UART_Transmit_DMA(&huart2, buf, SEND_DATA_LENGTH);

    if (req->rx_dma_req_bytes > 0) {
        while (int_tx_completed == 0) {
            __NOP();
        };
        int_rx_completed = 0;
        HAL_UART_Receive_DMA(&huart2, recv_buffer, req->rx_dma_req_bytes);
        while (int_rx_completed == 0) {
            __NOP();
        };

        memcpy(buf, recv_buffer, req->rx_dma_req_bytes);

        if (validate_msg(buf, req->rx_dma_req_bytes) < 0) {
            DBG_LOG("%s: error validate_msg\n", __func__);
            DBG_DUMP(buf, req->rx_dma_req_bytes);
            return;
        }

        memcpy(resp->rawdata, buf, req->rx_dma_req_bytes);
        resp->id = resp->rawdata[1] - '0';
        resp->type = resp->rawdata[2];
        resp->length = req->rx_dma_req_bytes - 4;
        resp->data = &resp->rawdata[3];

        handle_resp(resp);
    }
}

void internal_tx_task(void const *arg)
{
    internal_pool_q_id = osMailCreate(osMailQ(internal_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(internal_pool_q_id, osWaitForever);
        struct internal_msg_s *obj = (struct internal_msg_s *) event.value.p;
        handle_req(obj);
        osMailFree(internal_pool_q_id, obj);
    }
}

/* void internal_tx_task(void const *arg) */
/* { */
/*     internal_tx_pool_q_id = osMailCreate(osMailQ(internal_tx_pool_q), NULL); */

/*     while (1) { */
/*         osEvent event = osMailGet(internal_tx_pool_q_id, osWaitForever); */
/*         tx_received = (struct internal_tx_msg_s *) event.value.p; */
/*         handle_tx_msg(tx_received); */
/*         if (tx_received->rx_dma_req_bytes == -1) { */
/*             osMailFree(internal_tx_pool_q_id, tx_received); */
/*             tx_received = NULL; */
/*         } */
/*     } */
/* } */

void send_slot_id_req(uint8_t id)
{
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
    DoRejectSlot();

    uint8_t internal_id = id + 0x30;

    send_internal_req(id, CMD_SLOT_ID_REQ, &internal_id, 1);
}

static void handle_temperature(struct internal_rx_msg_s *msg)
{
    uni2Byte crc;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if ((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        uni4Byte *temp = &TestData.temperature[msg->id];
        for (int i = 0; i < 32; i++)
            (temp + i)->Float = *((float *)&msg->rawdata[i * 4 + 3]);
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    DoIncSlotIdStep(readSlotNumber);
}

static void handle_temerature_state(struct internal_rx_msg_s *msg)
{
    uint8_t count = 0;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    for (int i = 0; i < 32; i++)
        TestData.sensorState[readSlotNumber][i] = (LED_DIPLAY_MODE)msg->data[i];
}


static void handle_threshold_req(struct internal_rx_msg_s *msg)
{
    uni2Byte        crc;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        uni4Byte *threshold = &TestData.threshold[msg->id];
        for (int i = 0; i < 32; i++)
        {
            (threshold + i)->Float = *((float *)&msg->rawdata[i * 4 + 3]);
            /* TestData.threshold[readSlotNumber][i].UI8[0] = msg->rawdata[i * 4 + 3]; */
            /* TestData.threshold[readSlotNumber][i].UI8[1] = msg->rawdata[i * 4 + 4]; */
            /* TestData.threshold[readSlotNumber][i].UI8[2] = msg->rawdata[i * 4 + 5]; */
            /* TestData.threshold[readSlotNumber][i].UI8[3] = msg->rawdata[i * 4 + 6]; */
        }
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    // 초기화 하는 동안은 486 전송 하지 않는다, 초기화 중일때
    // startThreshold == TRUE 임.
    if (startThreshold != TRUE) {
        uint8_t thresholdData[130] = { 0 };
        thresholdData[0] = msg->id;
        memcpy(&thresholdData[1], &TestData.threshold[thresholdData[0]][0].UI8[0], 128);
        send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ, thresholdData, 129, 132, 152);
        /* doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ, thresholdData, 129, 132, 152); */
        /* SendUart485String(tx485DataDMA, 152); */
    }

    if(startThreshold == TRUE)
    {
        DoIncSlotIdStep(msg->id);
    }
}

static void handle_threshold_set(struct internal_rx_msg_s *msg)
{
    uni2Byte        crc;
    uint8_t         thresholdData[130];

    noReturnSendCt = 0;
    thresholdData[0] = msg->id;
    readSlotNumber = thresholdData[0];

    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        for(int inc = 0; inc < 32; inc++)
        {
            TestData.threshold[readSlotNumber][inc].UI8[0] = msg->rawdata[inc * 4 + 3];
            TestData.threshold[readSlotNumber][inc].UI8[1] = msg->rawdata[inc * 4 + 4];
            TestData.threshold[readSlotNumber][inc].UI8[2] = msg->rawdata[inc * 4 + 5];
            TestData.threshold[readSlotNumber][inc].UI8[3] = msg->rawdata[inc * 4 + 6];
        }
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    memcpy(&thresholdData[1], &TestData.threshold[readSlotNumber][0].UI8[0], 128);
    send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_SET, &thresholdData[0], 129, 132, 152);
    /* doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP, OP_WARNING_TEMP_SET, &thresholdData[0], 129, 132, 152); */
    /* SendUart485String(tx485DataDMA, 152); */
}
