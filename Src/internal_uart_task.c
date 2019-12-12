#include "cmsis_os.h"
#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"
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

int int_tx_completed;
int int_rx_completed;

static uint8_t recv_buffer[256];

static void DoAnsBoardType(struct internal_rx_msg_s *);
static void handle_threshold_req(struct internal_rx_msg_s *);
static void handle_threshold_set(struct internal_rx_msg_s *);
static void handle_temperature(struct internal_rx_msg_s *);
static void handle_temerature_state(struct internal_rx_msg_s *);
static void DoAnsRevisionApplySet(struct internal_rx_msg_s *);
static void DoAnsRevisionApplyReq(struct internal_rx_msg_s *);
static void DoAnsRevisionConstantSet(struct internal_rx_msg_s *);
static void DoAnsRevisionConstantReq(struct internal_rx_msg_s *);
static void DoAnsCalibrationNTCTableCal(struct internal_rx_msg_s *);
static void DoAnsCalibrationNTCTableReq(struct internal_rx_msg_s *);
static void DoAnsCalibrationNTCConstantSet(struct internal_rx_msg_s *);
static void DoAnsCalibrationNTCConstantReq(struct internal_rx_msg_s *);

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
    if (SysProperties.InterfaceStep == STEP_SLOT_ID)
        return -1;

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

    if (rawdata[0] != CMD_STX || rawdata[length - 1] != CMD_ETX)
        return -1;

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

static void handle_resp(struct internal_rx_msg_s *received)
{
    /* if (received->type == CMD_THRESHOLD_SET || received->type == CMD_THRESHOLD_REQ) { */
    /* DBG_LOG("int rx [%d] %s - (%d) ", */
    /*         received->id, cmd_str(received->type), received->length); */
    /* DBG_DUMP(received->data, received->length); */
    /* } */

    switch (received->type) {
    case CMD_BOARD_TYPE:
        DoAnsBoardType(received);
        break;
    case CMD_BOARD_EN_REQ:
        break;
    case CMD_BOARD_EN_SET:
        break;
    case CMD_SLOT_ID_REQ:
        noReturnSendCt = 0;
        if (SendSlotNumber == received->id) {
            SysProperties.slots[received->id].inserted = TRUE;
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
        DoAnsRevisionApplySet(received);
        break;
    case CMD_REVISION_CONSTANT_SET:
        DoAnsRevisionConstantSet(received);
        break;
    case CMD_REVISION_APPLY_REQ:
        DoAnsRevisionApplyReq(received);
        break;
    case CMD_REVISION_CONSTANT_REQ:
        DoAnsRevisionConstantReq(received);
        break;
    case CMD_CALIBRATION_NTC_CONSTANT_SET:
        DoAnsCalibrationNTCConstantSet(received);
        break;
    case CMD_CALIBRATION_NTC_CONSTANT_REQ:
        DoAnsCalibrationNTCConstantReq(received);
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
        DoAnsCalibrationNTCTableCal(received);
        break;
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        DoAnsCalibrationNTCTableReq(received);
        break;
    }
}

static void handle_req(struct internal_msg_s *obj)
{
    static uint8_t buf[160] = { 0 };
    struct internal_tx_msg_s *req = &obj->request;
    struct internal_rx_msg_s *resp = &obj->response;

    memset(buf, 0, sizeof(buf));

    doMakeSendSlotData(buf, req->id + 0x30, req->cmd,
                       req->data, req->length, SEND_DATA_LENGTH);
    noReturnSendCt++;

    /* if (req->cmd == CMD_THRESHOLD_SET || req->cmd == CMD_THRESHOLD_REQ) { */
        /* DBG_LOG("int tx [%d] %s: (%d) ", */
        /*         req->id, cmd_str(req->cmd), req->length); */
        /* DBG_DUMP(req->data, req->length); */
    /* } */

    int_tx_completed = 0;
    HAL_UART_Transmit_DMA(&huart2, buf, SEND_DATA_LENGTH);

    if (req->rx_dma_req_bytes > 0) {
        while (int_tx_completed == 0) {
            __NOP();
        };

        /* DBG_LOG("%s: rx_dma_req_bytes %d\n", __func__, req->rx_dma_req_bytes); */
        memset(buf, 0, 160);
        int_rx_completed = 0;
        HAL_UART_Receive_DMA(&huart2, recv_buffer, req->rx_dma_req_bytes);

        uint32_t old_tick = osKernelSysTick();
        while (int_rx_completed == 0) {
            __NOP();
            if (osKernelSysTick() - old_tick > 100) {
                DBG_LOG("int rx: %d slot %s is not responsed\n",
                        req->id, cmd_str(req->cmd));
                return;
            }
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

void internal_uart_task(void const *arg)
{
    internal_pool_q_id = osMailCreate(osMailQ(internal_pool_q), NULL);

    while (SysProperties.InterfaceStep == STEP_SLOT_ID)
        osDelay(1);

    while (1) {
        osEvent event = osMailGet(internal_pool_q_id, osWaitForever);
        struct internal_msg_s *obj = (struct internal_msg_s *) event.value.p;
        handle_req(obj);
        osMailFree(internal_pool_q_id, obj);
    }
}

static void DoAnsBoardType(struct internal_rx_msg_s *msg)
{
    if (msg)
        SysProperties.slots[msg->id].type = msg->data[0];
}

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
    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    for (int i = 0; i < 32; i++)
        TestData.sensorState[msg->id][i] = (LED_DIPLAY_MODE)msg->data[i];
}


static void handle_threshold_req(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if ((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        uni4Byte *threshold = &TestData.threshold[msg->id];
        for (int i = 0; i < 32; i++)
        {
            (threshold + i)->Float = *((float *)&msg->rawdata[i * 4 + 3]);
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
        memcpy(&thresholdData[1], &TestData.threshold[msg->id][0].UI8[0], 128);
        send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ, thresholdData, 129, 132, 152);
    }

    if (startThreshold == TRUE) {
        DoIncSlotIdStep(msg->id);
    }
}

static void handle_threshold_set(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    /* uni2Byte crc; */
    uint16_t calculated_crc = CRC16_Make(&msg->rawdata[1], 130);
    uint16_t received_crc = *((uint16_t *) &msg->rawdata[131]);

    /* if ((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132])) */
    if (calculated_crc == received_crc)
    {
        uni4Byte *threshold = &TestData.threshold[msg->id];
        for (int i = 0; i < 32; i++) {
            (threshold + i)->Float = *((float *)&msg->rawdata[i * 4 + 3]);
        }
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t data[130] = { 0 };
    data[0] = msg->id;
    memcpy(&data[1], &TestData.threshold[msg->id][0].UI8[0], 128);
    send_external_response(CMD_WARNING_TEMP, OP_WARNING_TEMP_SET, &data[0], 129, 132, 152);
}

static void DoAnsRevisionApplySet(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if ((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.revisionApply[msg->id] = msg->rawdata[3];
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t  data[2] = { 0 };
    data[0] = msg->id;
    data[1] = TestData.revisionApply[msg->id];
    send_external_response(CMD_REVISION, OP_REVISION_APPLY_SET, data, 2, 12, 32);
}

static void DoAnsRevisionApplyReq(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.revisionApply[msg->id] = msg->rawdata[3];
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t  data[2];
    data[0] = msg->id;
    data[1] = TestData.revisionApply[msg->id];

    send_external_response(CMD_REVISION, OP_REVISION_APPLY_REQ, data, 2, 12, 32);
}

static void DoAnsRevisionConstantSet(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if ((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.revisionConstant[msg->id].UI8[0] = msg->rawdata[3];
        TestData.revisionConstant[msg->id].UI8[1] = msg->rawdata[4];
        TestData.revisionConstant[msg->id].UI8[2] = msg->rawdata[5];
        TestData.revisionConstant[msg->id].UI8[3] = msg->rawdata[6];
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t  data[5] = { 0 };
    data[0] = msg->id;
    data[1] = TestData.revisionConstant[msg->id].UI8[0];
    data[2] = TestData.revisionConstant[msg->id].UI8[1];
    data[3] = TestData.revisionConstant[msg->id].UI8[2];
    data[4] = TestData.revisionConstant[msg->id].UI8[3];
    send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_SET, data, 6, 12, 32);
}

static void DoAnsRevisionConstantReq(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if ((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.revisionConstant[msg->id].UI8[0] = msg->rawdata[3];
        TestData.revisionConstant[msg->id].UI8[1] = msg->rawdata[4];
        TestData.revisionConstant[msg->id].UI8[2] = msg->rawdata[5];
        TestData.revisionConstant[msg->id].UI8[3] = msg->rawdata[6];
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t  data[5];
    data[0] = msg->id;
    data[1] = TestData.revisionConstant[msg->id].UI8[0];
    data[2] = TestData.revisionConstant[msg->id].UI8[1];
    data[3] = TestData.revisionConstant[msg->id].UI8[2];
    data[4] = TestData.revisionConstant[msg->id].UI8[3];
    send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_REQ, data, 6, 12, 32);
}

static void DoAnsCalibrationNTCTableCal(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte        crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        for (int i = 0; i < 32; i++)
        {
            osDelay(1);
            TestData.ntcCalibrationTable[msg->id][i].UI8[0] = msg->rawdata[i * 4 + 3];
            TestData.ntcCalibrationTable[msg->id][i].UI8[1] = msg->rawdata[i * 4 + 4];
            TestData.ntcCalibrationTable[msg->id][i].UI8[2] = msg->rawdata[i * 4 + 5];
            TestData.ntcCalibrationTable[msg->id][i].UI8[3] = msg->rawdata[i * 4 + 6];
        }
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t calData[130];
    calData[0] = msg->id;
    memcpy((void*)&calData[1],(void*)&TestData.ntcCalibrationTable[msg->id][0].UI8[0], 128);
    send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL, calData, 129, 132, 152);
}

static void DoAnsCalibrationNTCConstantSet(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if ((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.ntcCalibrationConst.UI8[0] = msg->rawdata[3];
        TestData.ntcCalibrationConst.UI8[1] = msg->rawdata[4];
        TestData.ntcCalibrationConst.UI8[2] = msg->rawdata[5];
        TestData.ntcCalibrationConst.UI8[3] = msg->rawdata[6];

        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET,
                           &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32);
}

static void DoAnsCalibrationNTCTableReq(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 130);

    if ((crc.UI8[0] == msg->rawdata[131]) && (crc.UI8[1] == msg->rawdata[132]))
    {
        for (int i = 0; i < 32; i++)
        {
            TestData.ntcCalibrationTable[msg->id][i].UI8[0] = msg->rawdata[i * 4 + 3];
            TestData.ntcCalibrationTable[msg->id][i].UI8[1] = msg->rawdata[i * 4 + 4];
            TestData.ntcCalibrationTable[msg->id][i].UI8[2] = msg->rawdata[i * 4 + 5];
            TestData.ntcCalibrationTable[msg->id][i].UI8[3] = msg->rawdata[i * 4 + 6];
        }
        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    uint8_t data[130] = { 0 };
    data[0] = msg->id;
    memcpy(&data[1], &TestData.ntcCalibrationTable[msg->id][0].UI8[0], 128);
    send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ, data, 129, 132, 152);
}

static void DoAnsCalibrationNTCConstantReq(struct internal_rx_msg_s *msg)
{
    if (!msg)
        return;

    noReturnSendCt = 0;
    readSlotNumber = msg->id;

    uni2Byte crc;
    crc.UI16 = CRC16_Make(&msg->rawdata[1], 8);

    if ((crc.UI8[0] == msg->rawdata[9]) && (crc.UI8[1] == msg->rawdata[10]))
    {
        TestData.ntcCalibrationConst.UI8[0] = msg->rawdata[3];
        TestData.ntcCalibrationConst.UI8[1] = msg->rawdata[4];
        TestData.ntcCalibrationConst.UI8[2] = msg->rawdata[5];
        TestData.ntcCalibrationConst.UI8[3] = msg->rawdata[6];

        crcErrorCount = 0;
    }
    else
    {
        crcErrorCount++;
    }

    send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ, &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32);
}
