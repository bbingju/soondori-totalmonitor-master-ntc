#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"
#include "0_StartSlotUartTask.h"
#include "0_soonFlashMemory.h"
#include "0_SdCard.h"

/*********************************************************************
*	Private variables
**********************************************************************/
__PACKED_STRUCT ext_rx_data_warning_temp_set_s {
    uint8_t slot_id;
    uint8_t channel;
    float temperature;
};

__PACKED_STRUCT ext_rx_msg_s {
    uint8_t cmd;
    uint8_t option;
    uint8_t ip[4];
    uint8_t data[22];
    uint8_t crc[2];
};

osMailQDef(ext_rx_pool_q, 8, struct ext_rx_msg_s);
osMailQId (ext_rx_pool_q_id);


__PACKED_STRUCT ext_tx_buffer_s {
    uint8_t raw[152];
    uint16_t bytes_to_transmit;
};

osMailQDef(ext_tx_pool_q, 18, struct ext_tx_buffer_s);
osMailQId (ext_tx_pool_q_id);

int ext_tx_completed;

static bool _validate_external_msg(uint8_t *rawdata, uint16_t length)
{
    if (rawdata[0] == RS_STX && rawdata[length - 1] == RS_ETX) {
        /* uint16_t received_crc = *((uint16_t *)&rawdata[length - 3]); */
        /* uint16_t calcurated_crc = CRC16_Make(&rawdata[1], length - 4); */

        /* if (received_crc != calcurated_crc) { */
        /*     DBG_LOG("%s: received_crc: %04x, calcurated_crc: %04x\n", */
        /*             __func__, received_crc, calcurated_crc); */
        /*     return false; */
        /* } */
        return true;
    }

    return false;
}

int push_external_rx(void *data, uint16_t length)
{
    if (length > 32 || data == NULL) {
        return -1;
    }

    if (!_validate_external_msg(data, length)) {
        DBG_LOG("%s: error validate_msg\n", __func__);
        DBG_DUMP(data, length);
        return -1;
    }

    struct ext_rx_msg_s *obj;
    obj = (struct ext_rx_msg_s *) osMailAlloc(ext_rx_pool_q_id, osWaitForever);
    if (!obj) {
        return -1;
    }
    memcpy(obj, data + 1, length - 4);
    osMailPut(ext_rx_pool_q_id, obj);

    return 0;
}


uint8_t         rx485DataDMA[256] = { 0 };      //dma 용도
/* uint8_t         Rx485Data[40] = { 0 };          //parsing 용도 */

/* uint8_t         tx485DataDMA[MAX_485_BUF_LEN] = { 0 };       //dma 용도 */

uint8_t         Rx485ReadCount = 0;
uint8_t         noReturn485SendCt = 0;

uint8_t 	ReadFileBuf[MAX_485_BUF_LEN];


//SD_CARD_STRUCT SdCard;
//TEST_STATE_STRUCT TestState;
uint8_t			FindFilelistFlag = 0;		//0 : 파일 리스트 검색 안하는중 / 1 : 파일 리스트 검색중


void SendUart485NonDma(uint8_t *data, uint16_t length)
{
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_UART_Transmit(&UART_RS485_HANDEL, data, length, 100);
    osDelay(100);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

static const char* cmd_str(uint8_t cmd)
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

static const char* option_str(uint8_t cmd, uint8_t option) {
    switch (cmd) {
    case CMD_TEMP_TEST:
        switch (option) {
        case OP_TEMP_START_RX:
            return "OP_TEMP_START_RX";
        case OP_TEMP_STOP:
            return "OP_TEMP_STOP";
        case OP_TEMP_SAMPLE_RATE:
            return "OP_TEMP_SAMPLE_RATE";
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
        case OP_SDCARD_LIST:
            return "OP_SDCARD_LIST";
        case OP_SDCARD_DOWNLOAD:
            return "OP_SDCARD_DOWNLOAD";
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

static void handle_rx_msg(struct ext_rx_msg_s *received)
{
    DBG_LOG("ext rx [%s::%s]: ", cmd_str(received->cmd),
            option_str(received->cmd, received->option));
    DBG_DUMP(received->data, 30);

    switch (received->cmd) {
    case CMD_TEMP_TEST:
        switch(received->option) {
        case OP_TEMP_START_RX:
            SysProperties.start_flag = TRUE;
            break;
        case OP_TEMP_STOP:
            SysProperties.start_flag = FALSE;
            break;
        case OP_TEMP_SAMPLE_RATE:
            doSaveIntervalTime(received);
            break;
        }
        break;
    case CMD_WARNING_TEMP:
        switch (received->option) {
        case OP_WARNING_TEMP_SET:
            CmdWarningTempSet(received);
            break;
        case OP_WARNING_TEMP_REQ:
            CmdWarningTempReq(received);
            break;
        }
        break;
    case CMD_REVISION:
        switch (received->option) {
        case OP_REVISION_APPLY_SET:
            CmdRevisionApplySet(received);
            break;
        case OP_REVISION_CONSTANT_SET:
            CmdRevisionConstantSet(received);
            break;
        case OP_REVISION_APPLY_REQ:
            CmdRevisionApplyReq(received);
            break;
        case OP_REVISION_CONSTANT_REQ:
            CmdRevisionConstantReq(received);
            break;
        }
        break;
    case CMD_CALIBRATION:
        switch (received->option) {
        case OP_CALIBRATION_RTD_CONSTANT_SET:
            CmdCalibrationRTDConstSet(received);
            break;
        case OP_CALIBRATION_NTC_CON_TABLE_CAL:
            CmdCalibrationNTCTableCal(received);
            break;
        case OP_CALIBRATION_NTC_CONSTANT_SET:
            CmdCalibrationNTCConstantSet(received);
            break;
        case OP_CALIBRATION_RTD_CONSTANT_REQ:
            CmdCalibrationRTDConstReq(received);
            break;
        case OP_CALIBRATION_NTC_CON_TABLE_REQ:
            CmdCalibrationNTCTableReq(received);
            break;
        case OP_CALIBRATION_NTC_CONSTANT_REQ:
            CmdCalibrationNTCConstantReq(received);
            break;
        }
        break;
    case CMD_SD_CARD:
        switch (received->option) {
        case OP_SDCARD_LIST:
            FindFilelistFlag = 1;
            DoReadFileList(received);
            FindFilelistFlag = 0;
            break;
        case OP_SDCARD_DOWNLOAD:
            DoSendFile(received);
            break;
        case OP_SDCARD_DELETE:
            break;
        case OP_SDCARD_FORMAT:
            break;
        case OP_SDCARD_ERROR:
            break;
        }
        break;
    case CMD_SLOT:
        switch (received->option) {
        case OP_SLOT_SET:
            break;
        case OP_SLOT_REQ:
            break;
        }
        break;
    case CMD_TIME:
        switch (received->option) {
        case OP_TIME_SET:
            doSetTime(received);
            break;
        case OP_TIME_REQ:
            doGetTime(received);
            break;
        }
        break;
    }
}

void external_rx_task(void const * argument)
{
    while (SysProperties.InterfaceStep == STEP_SLOT_ID)
        osDelay(1);

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
    HAL_UART_Receive_DMA(&huart1, rx485DataDMA, 32);

    /* //Task 부팅 완료 플레그 */
    /* SysProperties.bootingWate[1] = TRUE; */

    /* while(1) */
    /* { */
    /*     if( (SysProperties.bootingWate[0] == TRUE) &&   // 0 : StartDiaplayTask, */
    /*         (SysProperties.bootingWate[1] == TRUE) &&   // 1 : StartRs485Task, */
    /*         (SysProperties.bootingWate[2] == TRUE) &&   // 2 : StartSlotUartTask, */
    /*         (SysProperties.bootingWate[3] == TRUE) )    // 3 : StartRateTask */
    /*     { */
    /*         break; */
    /*     } */
    /*     osDelay(100); */
    /* } */

    ext_rx_pool_q_id = osMailCreate(osMailQ(ext_rx_pool_q), NULL);

    while (1) {
        osEvent event = osMailGet(ext_rx_pool_q_id, osWaitForever);
        struct ext_rx_msg_s *received = (struct ext_rx_msg_s *) event.value.p;
        handle_rx_msg(received);
        osMailFree(ext_rx_pool_q_id, received);
        HAL_UART_Receive_DMA(&huart1, rx485DataDMA, 32);
    }
}

void external_tx_task(void const * arg)
{
     HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

    ext_tx_pool_q_id = osMailCreate(osMailQ(ext_tx_pool_q), NULL);
    if (!ext_tx_pool_q_id) {
        DBG_LOG("%s: osMailCreate failed\n", __func__);
        return;
    }

    while (1) {
        osEvent event = osMailGet(ext_tx_pool_q_id, osWaitForever);
        struct ext_tx_buffer_s *to_transmit_ext = (struct ext_tx_buffer_s *) event.value.p;

        DBG_LOG("ext tx [%s::%s] (%d): ", cmd_str(to_transmit_ext->raw[1]),
                option_str(to_transmit_ext->raw[1], to_transmit_ext->raw[2]),
                to_transmit_ext->bytes_to_transmit);
        DBG_DUMP(to_transmit_ext->raw, to_transmit_ext->bytes_to_transmit);

        ext_tx_completed = 0;
        HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit_DMA(&UART_RS485_HANDEL, to_transmit_ext->raw, to_transmit_ext->bytes_to_transmit);

        while (ext_tx_completed == 0) {
            __NOP();
        };

        HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
        osMailFree(ext_tx_pool_q_id, to_transmit_ext);
    }
}


int send_external_response(uint8_t cmd, uint8_t option, void *data,
                           uint16_t data_write_len, uint16_t data_len, uint16_t buffer_len)
{
    /* if (length > 134 || id > 3) { */
    /*     return -1; */
    /* } */

    struct ext_tx_buffer_s *obj;
    obj = (struct ext_tx_buffer_s *) osMailAlloc(ext_tx_pool_q_id, osWaitForever);
    if (!obj) {
        DBG_LOG("%s: mail allocation failed\n", __func__);
        return -1;
    }

    doMakeSend485Data(obj->raw, cmd, option, data, data_write_len, data_len, buffer_len);
    obj->bytes_to_transmit = buffer_len;

    osMailPut(ext_tx_pool_q_id, obj);
    return 0;
}

/*********************************************************************
*	JumpToFunction
* 	각 명령별로 분기 시켜서 함수를 분리 함.
**********************************************************************/
/* void JumpToFunction485(void) */
/* { */
/*     //uint8_t i; */

/*     //SysProperties.FunctionHandlingFlag = BUSY; */

/*     if(Rx485Data[0] == RS_STX) */
/*     { */
/*         if(Rx485Data[31] == RS_ETX) */
/*         { */
/*             switch(Rx485Data[1]) */
/*             { */
/*             case CMD_TEMP_TEST: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_TEMP_START_RX: */
/*                     SysProperties.start_flag = TRUE; */
/*                     break; */
/*                 case OP_TEMP_STOP: */
/*                     SysProperties.start_flag = FALSE; */
/*                     break; */
/*                 case OP_TEMP_SAMPLE_RATE: */
/*                     doSaveIntervalTime(); */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_WARNING_TEMP: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_WARNING_TEMP_SET: */
/*                     CmdWarningTempSet(); */
/*                     break; */
/*                 case OP_WARNING_TEMP_REQ: */
/*                     CmdWarningTempReq(); */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_REVISION: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_REVISION_APPLY_SET: */
/*                     CmdRevisionApplySet(); */
/*                     break; */
/*                 case OP_REVISION_CONSTANT_SET: */
/*                     CmdRevisionConstantSet(); */
/*                     break; */
/*                 case OP_REVISION_APPLY_REQ: */
/*                     CmdRevisionApplyReq(); */
/*                     break; */
/*                 case OP_REVISION_CONSTANT_REQ: */
/*                     CmdRevisionConstantReq(); */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_CALIBRATION: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_CALIBRATION_RTD_CONSTANT_SET: */
/*                     CmdCalibrationRTDConstSet(); */
/*                     break; */
/*                 case OP_CALIBRATION_NTC_CON_TABLE_CAL: */
/*                     CmdCalibrationNTCTableCal(); */
/*                     break; */
/*                 case OP_CALIBRATION_NTC_CONSTANT_SET: */
/*                     CmdCalibrationNTCConstantSet(); */
/*                     break; */
/*                 case OP_CALIBRATION_RTD_CONSTANT_REQ: */
/*                     CmdCalibrationRTDConstReq(); */
/*                     break; */
/*                 case OP_CALIBRATION_NTC_CON_TABLE_REQ: */
/*                     CmdCalibrationNTCTableReq(); */
/*                     break; */
/*                 case OP_CALIBRATION_NTC_CONSTANT_REQ: */
/*                     CmdCalibrationNTCConstantReq(); */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_SD_CARD: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_SDCARD_LIST: */
/*                     FindFilelistFlag = 1; */
/*                     DoReadFileList(); */
/*                     FindFilelistFlag = 0; */
/*                     break; */
/*                 case OP_SDCARD_DOWNLOAD: */
/*                     DoSendFile(); */
/*                     break; */
/*                 case OP_SDCARD_DELETE: */
/*                     break; */
/*                 case OP_SDCARD_FORMAT: */
/*                     break; */
/*                 case OP_SDCARD_ERROR: */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_SLOT: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_SLOT_SET: */
/*                     break; */
/*                 case OP_SLOT_REQ: */
/*                     break; */
/*                 } */
/*                 break; */
/*             case CMD_TIME: */
/*                 switch(Rx485Data[2]) */
/*                 { */
/*                 case OP_TIME_SET: */
/*                     doSetTime(); */
/*                     break; */
/*                 case OP_TIME_REQ: */
/*                     doGetTime(); */
/*                     break; */
/*                 } */
/*                 break; */
/*             } */
/*         } */
/*     } */
/*     HAL_UART_Receive_DMA(&UART_RS485_HANDEL, rx485DataDMA, RS_READ_SIZE); */
/* } */

void DoSendFile(struct ext_rx_msg_s *msg)
{
    DoSendFileOpen(msg);
    DoSendFileBodyPacket(0x0000, (UINT)(16 * 1024));
    DoSendFileClose();
}

void DoSendFileOpen(struct ext_rx_msg_s *msg)
{
    uint8_t t[1] = {0};
    FRESULT res = FR_OK;
    uint8_t fileNameLen = 0;
    FILINFO fno;

    memset(sdValue.sendFileName, 0x00, sizeof(sdValue.sendFileName));
    sdValue.sendFileName[0] = '0';
    sdValue.sendFileName[1] = ':';

    for (sdValue.sendFileNameLen = 0; sdValue.sendFileNameLen < 22;
         sdValue.sendFileNameLen++) {
        if (msg->data[/* Rx485Data[7 +  */ sdValue.sendFileNameLen] == '>')
            break;

        sdValue.sendFileName[13 + sdValue.sendFileNameLen] =
            msg->data[/* Rx485Data[7 + */ sdValue.sendFileNameLen];
    }
    sdValue.sendFileName[2] = '/';
    sdValue.sendFileName[3] = '2';
    sdValue.sendFileName[4] = '0';
    sdValue.sendFileName[5] = sdValue.sendFileName[14];
    sdValue.sendFileName[6] = sdValue.sendFileName[15];
    sdValue.sendFileName[7] = '/';
    sdValue.sendFileName[8] = sdValue.sendFileName[16];
    sdValue.sendFileName[9] = sdValue.sendFileName[17];
    sdValue.sendFileName[10] = '/';
    sdValue.sendFileName[11] = sdValue.sendFileName[18];
    sdValue.sendFileName[12] = sdValue.sendFileName[19];

    res = f_stat((const TCHAR *)sdValue.sendFileName, &fno);

    /* if(osSemaphoreGetCount(CountingSem485TxHandle) == 0) */
    /*     osSemaphoreRelease(CountingSem485TxHandle); */

    if (res == FR_OK) {
        res = f_open(&sdValue.sendFileObject, sdValue.sendFileName,
                     FA_OPEN_EXISTING | FA_READ);
        if (res == FR_OK) //파일 정상 오픈
        {
            sdValue.sdState = SCS_OK;
            send_external_response(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_HEADER,
                                   sdValue.sendFileName, fileNameLen, 36, 56);
            /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD,
             * OP_SDCARD_DOWNLOAD_HEADER, sdValue.sendFileName, fileNameLen, 36,
             * 56); */
            /* SendUart485String(tx485DataDMA, 56); */
        } else //파일 오픈 에러
        {
            sdValue.sdState = SCS_OPEN_ERROR;
            t[0] = SCS_OPEN_ERROR;
            send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
            /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t,
             * 1, 12, 32); */
            /* SendUart485String(tx485DataDMA, 32); */
        }
    }
}

void DoSendFileBodyPacket(uint32_t Offset, UINT packetSize)
{
    /* uint8_t t[1] = {0}; */
    int32_t ReadSize;
    uint16_t i = 0;
    uni4Byte temp;

    /* if(osSemaphoreGetCount(CountingSem485TxHandle) == 0) */
    /*     osSemaphoreRelease(CountingSem485TxHandle); */

    do {
        memset(ReadFileBuf, 0x00, sizeof(ReadFileBuf));
        temp.UI32 = Offset + (packetSize * i);
        ReadFileBuf[0] = temp.UI8[0];
        ReadFileBuf[1] = temp.UI8[1];
        ReadFileBuf[2] = temp.UI8[2];
        ReadFileBuf[3] = temp.UI8[3];
        // util_mem_cpy(&ReadFileBuf[0], &temp.UI8[0], 4);
        temp.UI32 = Offset + (packetSize * (i + 1));
        ReadFileBuf[4] = temp.UI8[0];
        ReadFileBuf[5] = temp.UI8[1];
        ReadFileBuf[6] = temp.UI8[2];
        ReadFileBuf[7] = temp.UI8[3];
        // util_mem_cpy(&ReadFileBuf[4], &temp.UI8[0], 4);

        ReadSize = DoSendFileRead(Offset + (packetSize * i), packetSize);
        send_external_response(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_BADY,
                               &ReadFileBuf[0], ReadSize + 8, packetSize + 8,
                               packetSize + 8 + 20);
        /* doMakeSend485DataDownLoad(tx485DataDMA, CMD_SD_CARD,
         * OP_SDCARD_DOWNLOAD_BADY, &ReadFileBuf[0], ReadSize + 8, packetSize +
         * 8, packetSize + 8 + 20); */
        /* SendUart485String(&tx485DataDMA[0], packetSize + 8 + 20); */

        /*		uint16_t len = (packetSize + 8 + 20);
                        uint16_t ct  = (len / 32);

                        for(int j = 0; j < ct; j++)
                        {
                        SendUart485String(&tx485DataDMA[j * 32], 32);
                        len -= 32;
                        }
                        osDelay(1);
                        SendUart485String(&tx485DataDMA[j * 32], len);
        */
        if (packetSize != ReadSize) //마지막 페킷
        {
            osDelay(1);
            break;
        }
        i++;
    } while (1);

    osDelay(1);
}

void DoSendFileClose(void)
{
    FRESULT res = FR_OK;
    uint8_t t[1] = {0};

    res = f_close(&sdValue.sendFileObject);
    if (res == FR_OK) //파일 정상으로 닫힘
    {
        sdValue.sdState = SCS_OK;
        send_external_response(CMD_SD_CARD, OP_SDCARD_DOWNLOAD_FOOTER,
                               sdValue.sendFileName, sdValue.sendFileNameLen,
                               36, 56);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD,
         * OP_SDCARD_DOWNLOAD_FOOTER, sdValue.sendFileName,
         * sdValue.sendFileNameLen, 36, 56); */
        /* SendUart485String(tx485DataDMA, 56); */
    } else //파일 닫기 에러
    {
        sdValue.sdState = SCS_CLOSE_ERROR;
        t[0] = SCS_CLOSE_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1,
         * 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
    }
}

int32_t DoSendFileRead(FSIZE_t Offset, UINT ReadSize)
{
    FRESULT res = FR_OK;
    UINT br = 0;
    uint8_t t[1] = {0};

    res = f_lseek(&sdValue.sendFileObject, Offset);
    if (res != FR_OK) {
        sdValue.sdState = SCS_OK;
        t[0] = SCS_SEEK_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1,
         * 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
    }

    res = f_read(&sdValue.sendFileObject, &ReadFileBuf[8], ReadSize, &br);
    if (res != FR_OK) {
        sdValue.sdState = SCS_READ_ERROR;
        t[0] = SCS_READ_ERROR;
        send_external_response(CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
        /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1,
         * 12, 32); */
        /* SendUart485String(tx485DataDMA, 32); */
    }

    return (int32_t)br;
}

void DoReadFileList(struct ext_rx_msg_s *msg)
{
    uint8_t t[1] = {0};
    TCHAR root_directory[3] = "0:";

    send_external_response(CMD_SD_CARD, OP_SDCARD_LIST_START, t, 0, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_LIST_START, t, 0,
     * 12, 32); */
    /* SendUart485String(tx485DataDMA, 32); */

    memset(&sdValue.scanDir[0], 0x00, sizeof(sdValue.scanDir));
    osDelay(1);
    sdValue.scanDirDeep = 0;
    memset(&sdValue.scanFilePath, 0x00, sizeof(sdValue.scanFilePath));
    osDelay(1);

    if (msg->data[0] /* Rx485Data[7] == '.' */) //루트를 요청 했을 경우
    {
        osDelay(1);
        scan_files(root_directory);
    } else //경로 지정 했을 경우
    {
        memset(&sdValue.scanReadFileName[0], 0x00,
               sizeof(sdValue.scanReadFileName));
        sdValue.scanReadFileName[0] = '0';
        sdValue.scanReadFileName[1] = ':';

        for (int i = 0; i < 22; i++) {
            if (msg->data[/* Rx485Data[7 + */ i] == '>')
                break;

            sdValue.scanReadFileName[2 + i] = msg->data[/* Rx485Data[7 + */ i];
        }

        while (scan_files((char *)sdValue.scanReadFileName) != FR_OK) {
            osDelay(1);
        }
    }

    send_external_response(CMD_SD_CARD, OP_SDCARD_LIST_END, t, 0, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_LIST_END, t, 0,
     * 12, 32); */
    /* SendUart485String(tx485DataDMA, 32); */
}

void doSaveIntervalTime(struct ext_rx_msg_s *msg) //샘플레이트
{
    SysProperties.interval_ms = *((uint32_t *)msg->data);
    /* SysProperties.intervalTime.UI8[0] = msg->data[0]; //Rx485Data[ 7]; */
    /* SysProperties.intervalTime.UI8[1] = msg->data[1]; //Rx485Data[ 8]; */
    /* SysProperties.intervalTime.UI8[2] = msg->data[2]; //Rx485Data[ 9]; */
    /* SysProperties.intervalTime.UI8[3] = msg->data[3]; //Rx485Data[10]; */

    send_external_response(CMD_TEMP_TEST, OP_TEMP_SAMPLE_RATE,
                           &SysProperties.interval_ms, 4, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_SAMPLE_RATE,
     * &SysProperties.intervalTime.UI8[0], 4, 12, 32); */
    /* SendUart485String(tx485DataDMA, 32); */
}

void CmdWarningTempSet(struct ext_rx_msg_s *msg)
{
    struct ext_rx_data_warning_temp_set_s *d =
        (struct ext_rx_data_warning_temp_set_s *)&msg->data;

    if (!SysProperties.slots[d->slot_id].inserted)
        return;

    /* DBG_LOG("%s - slot %d, channel %d, temperature ", __func__, d->slot_id, */
    /*         d->channel); */
    /* DBG_DUMP(&d->temperature, 4); */

    send_internal_req(d->slot_id, CMD_THRESHOLD_SET, msg->data + 1, 5);
}

void CmdWarningTempReq(struct ext_rx_msg_s *msg)
{
    if (msg) {
        int id = msg->data[0];
        bool found = false;

        struct slot_properties_s *s;
        for (int i = 0; i < MAX_SLOT_NUM; i++) {
            s = &SysProperties.slots[i];
            if (s->id == id) {
                found = true;
                break;
            }
        }

        if (found)
            DoThresholdReq(s);
    }
}

void CmdRevisionApplySet(struct ext_rx_msg_s *msg)
{
    /* slot 번호 전달 , 0: 측정온도 모드, 1: 보정온도 모드 */
    DoRevisionApplySet(msg->data[0] /* Rx485Data[7] */, msg->data[1]/* Rx485Data[8] */);
}

void CmdRevisionConstantSet(struct ext_rx_msg_s *msg)
{
    uint8_t id = msg->data[0];

    TestData.revisionConstant[id].UI8[0] = msg->data[1]; //Rx485Data[ 8];
    TestData.revisionConstant[id].UI8[1] = msg->data[2]; //Rx485Data[ 9];
    TestData.revisionConstant[id].UI8[2] = msg->data[3]; //Rx485Data[10];
    TestData.revisionConstant[id].UI8[3] = msg->data[4]; //Rx485Data[11];

    DoRevisionConstantSet(id/* Rx485Data[7] */);
}

void CmdRevisionApplyReq(struct ext_rx_msg_s *msg)
{
    DoRevisionApplyReq(msg->data[0]/* Rx485Data[7] */);
}

void CmdRevisionConstantReq(struct ext_rx_msg_s *msg)
{
    DoRevisionConstantReq(msg->data[0]/* Rx485Data[7] */);
}

void CmdCalibrationRTDConstSet(struct ext_rx_msg_s *msg)
{
    uni4Byte readConst;
    uint8_t i = 0;

    TestData.rtdCalibrationConst.UI8[0] = msg->data[0]; // Rx485Data[ 7];
    TestData.rtdCalibrationConst.UI8[1] = msg->data[1]; // Rx485Data[ 8];
    TestData.rtdCalibrationConst.UI8[2] = msg->data[2]; // Rx485Data[ 9];
    TestData.rtdCalibrationConst.UI8[3] = msg->data[3]; // Rx485Data[10];

    readConst.UI32 = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);

    if (TestData.rtdCalibrationConst.Float != readConst.Float) {
        do {
            doFlashWriteRevision();
            readConst.UI32 = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);
            i++;
            if (i > 10)
                break;
        } while (TestData.rtdCalibrationConst.Float != readConst.Float);
    }

    if (1 > 10) {
        i = 0xFF;
        send_external_response(CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET,
                               &i, 1, 12, 32);
    } else {
        send_external_response(CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET,
                               &readConst.UI8[0], 4, 12, 32);
    }

    /* if(i > 10)		//기록 실패 */
    /* { */
    /*     i = 0xFF; */
    /*     doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
     * OP_CALIBRATION_RTD_CONSTANT_SET, &i, 1, 12, 32); */
    /* } */
    /* else			//기록 성공 */
    /* { */
    /*     doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION,
     * OP_CALIBRATION_RTD_CONSTANT_SET, &readConst.UI8[0], 4, 12, 32); */
    /* } */
    /* SendUart485String(tx485DataDMA, 32); */
}

void CmdCalibrationNTCTableCal(struct ext_rx_msg_s *msg)
{
    DoCalibrationNTCTableCal(msg->data[0]/* Rx485Data[7] */);	//slot 번호 전달
}

void CmdCalibrationNTCConstantSet(struct ext_rx_msg_s *msg)
{
    TestData.ntcCalibrationConst.UI8[0] = msg->data[1]; //Rx485Data[ 8];
    TestData.ntcCalibrationConst.UI8[1] = msg->data[2]; //Rx485Data[ 9];
    TestData.ntcCalibrationConst.UI8[2] = msg->data[3]; //Rx485Data[10];
    TestData.ntcCalibrationConst.UI8[3] = msg->data[4]; //Rx485Data[11];

    DoCalibrationNTCConstantSet(msg->data[0]/* Rx485Data[ 7] */);
}

void CmdCalibrationRTDConstReq(struct ext_rx_msg_s *msg)
{
    send_external_response(CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET,
                           &TestData.rtdCalibrationConst.UI8[0], 4, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET, &TestData.rtdCalibrationConst.UI8[0], 4, 12, 32); */
    /* SendUart485String(tx485DataDMA, 32); */
}

void CmdCalibrationNTCTableReq(struct ext_rx_msg_s *msg)
{
    uint8_t id = msg->data[0];
    DoCalibrationNTCTableReq(id); //slot 번호 전달
    /* DoCalibrationNTCTableReq(Rx485Data[7]); //slot 번호 전달 */
}

void CmdCalibrationNTCConstantReq(struct ext_rx_msg_s *msg)
{
    uint8_t id = msg->data[0];
    /* DoCalibrationNTCConstantReq(Rx485Data[7]); //slot 번호 전달 */
    DoCalibrationNTCConstantReq(id); //slot 번호 전달
}

void doSetTime(struct ext_rx_msg_s *msg)
{
    //    uint8_t i = 0;
    uint8_t res = TRUE;
    HAL_StatusTypeDef resHal;
    RTC_DateTypeDef setDate, getDate;
    RTC_TimeTypeDef setTime, getTime;

    setDate.Year = msg->data[0];    // Rx485Data[7];
    setDate.Month = msg->data[1];   // Rx485Data[8];
    setDate.Date = msg->data[2];    // Rx485Data[9];
    setDate.WeekDay = msg->data[6]; // Rx485Data[13];
    setTime.Hours = msg->data[3];   // Rx485Data[10];
    setTime.Minutes = msg->data[4]; // Rx485Data[11];
    setTime.Seconds = msg->data[5]; // Rx485Data[12];
    setTime.TimeFormat = RTC_HOURFORMAT12_AM;
    setTime.StoreOperation = RTC_STOREOPERATION_RESET;

    do {
        while (1) {
            resHal = HAL_RTC_SetTime(&hrtc, &setTime, RTC_FORMAT_BIN);
            if (resHal == HAL_OK)
                break;
            osDelay(1);
        }
        while (1) {
            resHal = HAL_RTC_SetDate(&hrtc, &setDate, RTC_FORMAT_BIN);
            if (resHal == HAL_OK)
                break;
            osDelay(1);
        }

        HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);

        /*i++;
          if(i > 100)
          {
          res = FALSE;
          break;
          }*/

        if ((setDate.Year == getDate.Year) &&
            (setDate.Month == getDate.Month) &&
            (setDate.Date == getDate.Date) &&
            (setTime.Hours == getTime.Hours) &&
            (setTime.Minutes == getTime.Minutes) &&
            (setTime.Seconds == getTime.Seconds)) {
            break;
        }
    } while (1);

    send_external_response(CMD_TIME, OP_TIME_SET, &res, 1, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_TIME, OP_TIME_SET, &res, 1, 12, 32);
     */
    /* SendUart485String(tx485DataDMA, 32); */
}

void doGetTime(struct ext_rx_msg_s *msg)
{
    uint8_t res = 0;

    send_external_response(CMD_TIME, OP_TIME_REQ, &res, 0, 12, 32);
    /* doMakeSend485Data(tx485DataDMA, CMD_TIME, OP_TIME_REQ, &res, 0, 12, 32);
     */
    /* SendUart485String(tx485DataDMA, 32); */
}
