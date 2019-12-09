#include "cmsis_os.h"
#include "0_Util.h"
#include "internal_rx_task.h"
#include <string.h>
#include "0_StartSlotUartTask.h"
#include "0_StartRs485Task.h"

extern uint8_t readSlotNumber;
extern uint8_t crcErrorCount;
extern uint8_t startThreshold;
extern uint8_t SendSlotNumber;
extern uint8_t noReturnSendCt;


struct internal_rx_msg_s {
    uint8_t id;
    uint8_t type;
    uint16_t length;
    uint8_t rawdata[160];
    uint8_t *data;
};

osMailQDef(internal_rx_pool_q, 1, struct internal_rx_msg_s);
osMailQId (internal_rx_pool_q_id);

static void handle_threshold_req(struct internal_rx_msg_s *msg);
static void handle_threshold_set(struct internal_rx_msg_s *msg);
static void handle_temperature(struct internal_rx_msg_s *msg);
static void handle_temerature_state(struct internal_rx_msg_s *msg);

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
    obj->id = obj->rawdata[1] - '0';
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

static void handle_rx_msg(struct internal_rx_msg_s *received)
{
    if (received->type == CMD_THRESHOLD_SET || received->type == CMD_THRESHOLD_REQ) {
    /* print_bytes(received->rawdata, received->length + 4); */
    DBG_LOG("int rx [%d] %s: (%d) ",
            received->id, cmd_str(received->type), received->length);
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

void internal_rx_task(void const *arg)
{
    internal_rx_pool_q_id = osMailCreate(osMailQ(internal_rx_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(internal_rx_pool_q_id, osWaitForever);
        struct internal_rx_msg_s *received = (struct internal_rx_msg_s *) event.value.p;
        handle_rx_msg(received);
        osMailFree(internal_rx_pool_q_id, received);
    }
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

    if((crc.UI8[0] == &msg->rawdata[131]) && (crc.UI8[1] == &msg->rawdata[132]))
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
