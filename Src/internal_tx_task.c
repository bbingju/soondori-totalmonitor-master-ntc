#include "cmsis_os.h"
#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "internal_tx_task.h"
#include <string.h>


struct internal_tx_msg_s {
    uint8_t id;
    uint8_t type;
    uint16_t length;
    uint8_t rawdata[134];
    uint8_t *data;
};

osMailQDef(internal_tx_pool_q, 8, struct internal_tx_msg_s);
osMailQId (internal_tx_pool_q_id);

static int validate_msg(uint8_t *rawdata, uint16_t length)
{

    if (rawdata[0] == CMD_STX && rawdata[length - 1] == CMD_ETX)
        return 0;

    return -1;
}

int internal_tx_msg_push(uint8_t id, uint8_t cmd, void *data, uint16_t length)
{
    if (length > 134 || data == NULL) {
        return -1;
    }

    struct internal_tx_msg_s *obj;
    obj = (struct internal_tx_msg_s *) osMailAlloc(internal_tx_pool_q_id, osWaitForever);
    if (!obj) {
        return -1;
    }

    memcpy(obj->rawdata, data, length);
    obj->id = obj->rawdata[1];
    obj->type = obj->rawdata[2];
    obj->length = length - 4;
    obj->data = &obj->rawdata[3];

    osMailPut(internal_tx_pool_q_id, obj);
    return 0;
}

static void handle_msg(struct internal_tx_msg_s *received)
{
    static uint8_t buf[SEND_DATA_LENGTH] = { 0 };

    memset(buf, 00, sizeof(buf));
    doMakeSendSlotData(buf, received->id + 0x30, received->type,
                       &received->id, received->length, SEND_DATA_LENGTH);
}

void internal_tx_task(void const *arg)
{
    internal_tx_pool_q_id = osMailCreate(osMailQ(internal_tx_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(internal_tx_pool_q_id, osWaitForever);
        struct internal_tx_msg_s *received = (struct internal_tx_msg_s *) event.value.p;
        handle_msg(received);
        osMailFree(internal_tx_pool_q_id, received);
    }
}
