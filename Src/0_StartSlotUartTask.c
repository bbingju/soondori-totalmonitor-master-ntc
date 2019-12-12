#include "math.h"
#include "0_StartSlotUartTask.h"
#include "0_GlobalValue.h"
#include "0_UartCallback.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"
#include <string.h>

/*********************************************************************
*       Private variables
**********************************************************************/
//UPDATA_FLAG     Updata_Flag;

GPIO_TypeDef *  SLAVE_CS_PORT[4] = {SLAVE_CS0_GPIO_Port,  SLAVE_CS1_GPIO_Port,  SLAVE_CS2_GPIO_Port,  SLAVE_CS3_GPIO_Port}; //todo : 순서를 0123 으로 바꿔야함
uint16_t        SLAVE_CS_PIN[4]  = {SLAVE_CS0_Pin,        SLAVE_CS1_Pin,        SLAVE_CS2_Pin,        SLAVE_CS3_Pin};       // back plate 의 컨넥터가 잘못 되어 있음

uint8_t         TxDataBuffer[UART_TX_BUF_MAX] = { 0 };  //dma 용도
uint8_t         RxDataDMA[134 * 10]  = { 0 };                               //dma 용도

/* uint8_t         RxSlotData[140]  = { 0 };                       //parsing 용도 */
uint8_t         RxSlotDataLength = 0;
uint8_t         RxReadCount              = 0;

uint8_t         ReceveDataLength = 0;
uint8_t         SendSlotNumber   = 0;
uint8_t         SendChannel      = 0;
uint8_t         SlotCheck                = FALSE;
uint8_t         noReturnSendCt   = 0;

uni4Byte        readTemp;
uint8_t         readSlotNumber;

uint8_t         temp;
uint8_t         crcErrorCount    = 0;

uint8_t                 tempReqFlag              = FALSE;
uint8_t                 semCount                 = 0;
uint8_t                 startThreshold   = FALSE;
/* RX_QUEUE_STRUCT RxQueue; */

extern int int_tx_completed;
extern int int_rx_completed;
extern IWDG_HandleTypeDef hiwdg;

void check_slots_inserted(struct slot_properties_s *slots, int num_of_slots)
{
    uint8_t buf[150] = { 0 };

    for (int i = 0; i < num_of_slots; i++) {
        for (int j = 0; j < 11; j++) {
            HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

            doMakeSendSlotData(buf, slots[i].id + 0x30, CMD_TEMP_REQ,
                       buf, 0, SEND_DATA_LENGTH);
            noReturnSendCt++;
            int_tx_completed = 0;
            HAL_UART_Transmit_DMA(&huart2, buf, SEND_DATA_LENGTH);
            while (int_tx_completed == 0) {
                __NOP();
            };

            int_rx_completed = 0;
            HAL_UART_Receive_DMA(&huart2, buf, 134);

            /* DBG_LOG("slot %d, %d times\n", i, j); */

            uint32_t old_tick = osKernelSysTick();
            while (int_rx_completed == 0) {
                __NOP();
                if (osKernelSysTick() - old_tick > 100)
                    break;
            };

            if (int_rx_completed)
                noReturnSendCt = 0;
        }

        slots[i].inserted = noReturnSendCt > 9 ? false : true;
        noReturnSendCt = 0;
    }

    for (int i = 0; i < num_of_slots; i++) {
        DBG_LOG("slot %d inserted %s\n", slots[i].id,
                slots[i].inserted ? "TRUE" : "FALSE");
    }
}

void system_task(void const * argument)
{
    SysProperties.InterfaceStep = STEP_SLOT_ID;

    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_OE1_GPIO_Port, SLAVE_OE1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(UART_EN_BT_GPIO_Port, UART_EN_BT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_SET);

    DoSlotReset(ALL_SLOT);
    DoRejectSlot();
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(100);
    HAL_IWDG_Refresh(&hiwdg);
    //todo : 센서 타입을 슬레이브 보드에서 읽어 와야 함.

    for (;;)
    {
        switch (SysProperties.InterfaceStep) {

        case STEP_SLOT_ID: {// 부팅 하면 각 슬롯의 id 를 지정 한다. id 는 '0'에서 시작한다.
            volatile bool existed_any_slot = false;

            while (!existed_any_slot) {
                check_slots_inserted(SysProperties.slots, MAX_SLOT_NUM);
                HAL_IWDG_Refresh(&hiwdg);
                FOREACH(struct slot_properties_s *s, SysProperties.slots) {
                  if (s->inserted)
                    existed_any_slot = true;
                }
                /* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
                /*     struct slot_properties_s *slot = &SysProperties.slots[i]; */
                /*     if (slot->inserted) { */
                /*         existed_any_slot = true; */
                /*         break; */
                /*     } */
                /* } */
            }
        /* if (system_reset_needed) { */
        /*     HAL_IWDG_Refresh(&hiwdg); */
        /*     osDelay(500); */
        /*     HAL_NVIC_SystemReset(); */
        /* } */

        SysProperties.InterfaceStep = STEP_READ_THRESHOLD;
      }
        break;

      case STEP_READ_THRESHOLD: //각 슬롯의 경고 온도 값을 불러 온다.
        startThreshold = TRUE;
        FOREACH(struct slot_properties_s *s, SysProperties.slots) {
          DoThresholdReq(s);
        }
        /* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
        /*   struct slot_properties_s *slot = &SysProperties.slots[i]; */
        /*   DoThresholdReq(slot); */
        /* } */
        osDelay(1000);
        SysProperties.InterfaceStep = STEP_TEMP_READ;
        break;

      case STEP_TEMP_READ:  // 각 슬롯의 id 설정 완료 후 온도센서의 온도를 요청 한다.
        FOREACH(struct slot_properties_s *s, SysProperties.slots) {
          DoReqTemperature(s);
          DoReqTemperatureState(s);
          osDelay(SysProperties.interval_ms - 50);
        }
        /* for (int i = 0; i < MAX_SLOT_NUM; i++) { */
        /*     struct slot_properties_s *slot = &SysProperties.slots[i]; */
        /*     DBG_LOG("slot->id: %d, type: %d, inserted: %d\n", slot->id, slot->type, slot->inserted); */
        /*     DoReqTemperature(slot); */
        /*     DoReqTemperatureState(slot); */
        /*     osDelay(SysProperties.interval_ms - 50); */
        /* } */

        if (noReturnSendCt > 10)
          noReturnSendCt = 0;
        break;
    }
  }
}

/*********************************************************************
*       doSlotReset
*       SLOT 통신 문제가 발생 됬을 때 SLOT 강제 리셋 시킴
*   slot : SLOT 번호, ALL_SLOT : 모든 SLOT RESET
**********************************************************************/
void DoSlotReset(uint8_t slot)
{
/*    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);      //buffer ic on

    if(slot == ALL_SLOT)
    {
        HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SLAVE_CS_PORT[slot], SLAVE_CS_PIN[slot], GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_SET);
    HAL_Delay(70);
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_RESET);
    */
}

/*********************************************************************
*       doSelectSlot
*       SLOT 선택 하는 함수
*   slot : SLOT 번호
**********************************************************************/
void DoSelectSlot(uint8_t slot)
{
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, slot == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, slot == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, slot == 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, slot == 3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*********************************************************************
*       doRejectSlot
*       SLOT 선택 제거 함수
**********************************************************************/
void DoRejectSlot(void)
{
    HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
**********************************************************************
*
*       Tx 관련 함수
*
*   doSet 으로 시작 하는 함수 : slot에 설정을 하는 함수
*   doReq 으로 시작 하는 함수 : slot에 값을 요청 하는 함수
**********************************************************************
**********************************************************************/

/*********************************************************************
*       txFunction
*       TX 전송 관리 함수
*   slotNumber : SLOT 지정
**********************************************************************/
void UartInternalTxFunction(uint8_t* datas, uint16_t length)
{
    //TX FUNCTION
    /* if(BinarySemSlaveTxHandle != NULL)  //TX */
    /* { */
    /*     if(osSemaphoreWait(BinarySemSlaveTxHandle, 0) == osOK) */
    /*     { */
    /*         // 온도값 요청 */
    /*         noReturnSendCt++; */
    /*         HAL_UART_Transmit_DMA(&BLUETOOTH_HANDEL, datas, length); */
    /*         osDelay(1); */
    /*     } */
    /* } */
}

void DoCalibrationNTCTableCal(uint8_t slotNumber)
{
    send_internal_req(slotNumber, CMD_CALIBRATION_NTC_CON_TABLE_CAL, &TestData.mainBoard[MBS_RTD].UI8[0], 4);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, (slotNumber + 0x30), CMD_CALIBRATION_NTC_CON_TABLE_CAL, &TestData.mainBoard[MBS_RTD].UI8[0], 4, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 134);	// 응답은 134로 들어온다. */
    /* osDelay(100); */
}

void DoCalibrationNTCConstantSet(uint8_t slotNumber)
{
    send_internal_req(slotNumber, CMD_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConst.UI8[0], 4);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, (slotNumber + 0x30), CMD_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConst.UI8[0], 4, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
    /* osDelay(100); */
}

void DoCalibrationNTCTableReq(uint8_t slotNumber)
{
    send_internal_req(slotNumber, CMD_CALIBRATION_NTC_CON_TABLE_REQ, NULL, 0);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, (slotNumber + 0x30), CMD_CALIBRATION_NTC_CON_TABLE_REQ, &slotNumber, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 134);      // 응답은 134로 들어온다. */
    /* osDelay(100); */
}

void DoCalibrationNTCConstantReq(uint8_t slotNumber)
{
    send_internal_req(slotNumber, CMD_CALIBRATION_NTC_CONSTANT_REQ, NULL, 0);

    /* memset((void*)&TxDataBuffer[0], 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, (slotNumber + 0x30), CMD_CALIBRATION_NTC_CONSTANT_REQ, &slotNumber, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
    /* osDelay(100); */
}

void DoThresholdSet(struct slot_properties_s *slot, uint8_t channal, uni4Byte thresholdTemp)
{
    if (slot && slot->inserted) {
        uint8_t u[5] = {0};

        u[0] = channal;
        u[1] = thresholdTemp.UI8[0];
        u[2] = thresholdTemp.UI8[1];
        u[3] = thresholdTemp.UI8[2];
        u[4] = thresholdTemp.UI8[3];

        send_internal_req(slot->id, CMD_THRESHOLD_SET, u, 5);
    }
    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_THRESHOLD_SET, u, 5, SEND_DATA_LENGTH); */
    /* /\* DBG_LOG("%s: ", __func__); *\/ */
    /* /\* print_bytes(TxDataBuffer, SEND_DATA_LENGTH); *\/ */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 134);  // 응답은 134로 들어온다. */
    /* osDelay(100); */
}

void DoThresholdReq(struct slot_properties_s *slot)
{
    if (slot && slot->inserted) {
        DBG_LOG("%s slot %d\n", __func__, slot->id);
        send_internal_req(slot->id, CMD_THRESHOLD_REQ, NULL, 0);
    }

    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_THRESHOLD_REQ, &slotNumber, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 134);  // 응답은 134로 들어온다. */
    /* osDelay(100); */
}

/*********************************************************************
*       doReqSlotID
*       SLOT 각 보드의 ID 를 설정
*   slotNumber : SLOT 지정
**********************************************************************/
void DoReqSlotID(uint8_t slotNumber)
{
    HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
    DoRejectSlot();

    uint8_t id = slotNumber + 0x30;
    send_internal_req(slotNumber, CMD_SLOT_ID_REQ, &id, sizeof(id));

    /* uint8_t u[1] = {0}; */
    /* u[0] = slotNumber + 0x30; */
    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, u[0], CMD_SLOT_ID_REQ, u, 1, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
}


/*********************************************************************
*       doReqTemperature
*       슬롯에 센서 온도를 요청 함, 전송 받는 값은 flot(4byte)으로 한다.
*   slotNumber : SLOT 지정
**********************************************************************/
void DoReqTemperature(struct slot_properties_s *slot)
{
    if (slot && slot->inserted) {
        HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
        send_internal_req(slot->id, CMD_TEMP_REQ, NULL, 0);
    }

    /* uint8_t u[1] = {0}; */
    /* slotNumber += 0x30; */
    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* doMakeSendSlotData(TxDataBuffer, slotNumber, CMD_TEMP_REQ, u, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 134); */
}

void DoReqTemperatureState(struct slot_properties_s *slot)
{
    /* uint8_t u[1] = {0}; */

    if (slot && slot->inserted) {
        HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);
        send_internal_req(slot->id, CMD_TEMP_STATE_REQ, NULL, 0);
    }

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */
    /* slotNumber += 0x30; */
    /* doMakeSendSlotData(TxDataBuffer, slotNumber, CMD_TEMP_STATE_REQ, u, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 38); */
}


void DoRevisionApplySet(uint8_t slotNumber, uint8_t mode)		//slot 번호 전달 , 0: 측정온도 모드, 1: 보정온도 모드
{
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    send_internal_req(slotNumber, CMD_REVISION_APPLY_SET, &mode, 1);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */

    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_REVISION_APPLY_SET, &mode, 1, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
}

void DoRevisionConstantSet(uint8_t slotNumber)
{
    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    send_internal_req(slotNumber, CMD_REVISION_CONSTANT_SET, TestData.revisionConstant[slotNumber].UI8, 4);

    /* memset((void*)&TxDataBuffer[0], 0x00, sizeof(TxDataBuffer)); */

    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_REVISION_CONSTANT_SET, &TestData.revisionConstant[slotNumber].UI8[0], 4, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
}

void DoRevisionApplyReq(uint8_t slotNumber)
{

    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    send_internal_req(slotNumber, CMD_REVISION_APPLY_REQ, NULL, 0);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */

    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_REVISION_APPLY_REQ, &slotNumber, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
}

void DoRevisionConstantReq(uint8_t slotNumber)
{

    HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_RESET);

    send_internal_req(slotNumber, CMD_REVISION_CONSTANT_REQ, NULL, 0);

    /* memset(TxDataBuffer, 0x00, sizeof(TxDataBuffer)); */

    /* doMakeSendSlotData(TxDataBuffer, (uint8_t)(slotNumber + 0x30), CMD_REVISION_CONSTANT_REQ, &slotNumber, 0, SEND_DATA_LENGTH); */
    /* UartInternalTxFunction(TxDataBuffer, SEND_DATA_LENGTH); */
    /* HAL_UART_Receive_DMA(&huart2, RxDataDMA, 12); */
}

/*********************************************************************
*       doIncSlotIdStep
*       슬롯이 4개(차후 6개로 증가 예정)를 순차적으로 요청 하시 위한 함수
*   slotNumber : SLOT 지정
**********************************************************************/
void DoIncSlotIdStep(uint8_t slotNumber)
{
    uint8_t ct = 0;

    switch(SysProperties.InterfaceStep)
    {
    case STEP_SLOT_ID:
        SendSlotNumber = slotNumber + 1;
        if(SendSlotNumber > 3)  {
            SendSlotNumber = 0;
            SysProperties.InterfaceStep  = STEP_READ_THRESHOLD;
            /* RxQueue_Clear(&RxQueue); */

            bool system_reset_needed = true;
            for (int i = 0; i < 4; i++) {
                if (SysProperties.slots[i].inserted) {
                    system_reset_needed = false;
                    break;
                }
            }
            if (system_reset_needed)
                HAL_NVIC_SystemReset();
        }
        break;
    case STEP_READ_THRESHOLD:                       //각 슬롯의 경고 온도 값을 불러 온다.
        do{
            SendSlotNumber = ++slotNumber;
            if(SendSlotNumber > 3)  {
                SendSlotNumber = 0;
                SysProperties.InterfaceStep = STEP_TEMP_READ;
                /* RxQueue_Clear(&RxQueue); */
            }
            ct++;
            if(ct > 3)      break;
        } while (!SysProperties.slots[SendSlotNumber].inserted);
        break;
    case STEP_TEMP_READ:
        do{
            SendSlotNumber = ++slotNumber;
            if(SendSlotNumber > 3)  {
                SendSlotNumber = 0;
                startThreshold = FALSE;
            }
            ct++;
            if(ct > 3) break;
        } while (!SysProperties.slots[SendSlotNumber].inserted);
        break;
    }
}

/*********************************************************************
**********************************************************************
*
*       Rx 관련 함수
*
*   doAns 으로 시작 하는 함수 : 요청한 응답이 있거나, 설정 완료에 대한 응답
**********************************************************************
**********************************************************************/

/*********************************************************************
*       doSlotJumpFunction
*       Rx Data 를 Parsing 하기 위한 분기 함수
**********************************************************************/
/* void DoSlotJumpFunction(void) */
/* { */
/*     if(RxSlotData[0] == CMD_STX) */
/*     { */
/*         //if(RxSlotData[11] == CMD_ETX) */
/*         if(RxReadCount == 11) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_BOARD_TYPE: */
/*                 DoAnsBoardType(); */
/*                 break; */
/*             case CMD_BOARD_EN_REQ: */
/*                 break; */
/*             case CMD_BOARD_EN_SET: */
/*                 break; */
/*             case CMD_SLOT_ID_REQ: */
/*                 DoAnsReqSlotID(); */
/*                 break; */
/*             case CMD_HW_VER: */
/*                 break; */
/*             case CMD_FW_VER: */
/*                 break; */
/*             case CMD_UUID_REQ: */
/*                 break; */
/*             case CMD_ADC_REQ: */
/*                 break; */
/*             case CMD_RELAY_REQ: */
/*                 break; */
/*             case CMD_RELAY_SET: */
/*                 break; */
/*             case CMD_REVISION_APPLY_SET: */
/*                 DoAnsRevisionApplySet(); */
/*                 break; */
/*             case CMD_REVISION_CONSTANT_SET: */
/*                 DoAnsRevisionConstantSet(); */
/*                 break; */
/*             case CMD_REVISION_APPLY_REQ: */
/*                 DoAnsRevisionApplyReq(); */
/*                 break; */
/*             case CMD_REVISION_CONSTANT_REQ: */
/*                 DoAnsRevisionConstantReq(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CONSTANT_SET: */
/*                 DoAnsCalibrationNTCConstantSet(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CONSTANT_REQ: */
/*                 DoAnsCalibrationNTCConstantReq(); */
/*                 break; */
/*             } */
/*         } */
/*         //else if(RxSlotData[37] == CMD_ETX) */
/*         else if(RxReadCount == 37) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_TEMP_STATE_REQ: */
/*                 DoAnsTemperatureState(); */
/*                 break; */
/*             } */
/*         } */
/*         //else if(RxSlotData[133] == CMD_ETX) */
/*         else if(RxReadCount == 133) */
/*         { */
/*             switch(RxSlotData[2]) */
/*             { */
/*             case CMD_TEMP_REQ: */
/*                 DoAnsTemperature(); */
/*                 break; */
/*             case CMD_THRESHOLD_REQ: */
/*                 DoAnsThresholdReq(); */
/*                 break; */
/*             case CMD_THRESHOLD_SET: */
/*                 DoAnsThresholdSet(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CON_TABLE_CAL: */
/*                 DoAnsCalibrationNTCTableCal(); */
/*                 break; */
/*             case CMD_CALIBRATION_NTC_CON_TABLE_REQ: */
/*                 DoAnsCalibrationNTCTableReq(); */
/*                 break; */
/*             } */
/*         } */
/*     } */
/* } */

/*********************************************************************
*       doAnsBoardType
*       보드 타입 응답 함수
**********************************************************************/
/* void DoAnsBoardType(struct internal_rx_msg_s *msg) */
/* { */
/*   if (msg) */
/*     SysProperties.slotType[msg->id] = msg->data[0]; */
/* } */

/*********************************************************************
*       doAnsReqSlotID
*       슬롯 id 설정 확인 함수
**********************************************************************/
/* void DoAnsReqSlotID(void) */
/* { */
/*     noReturnSendCt = 0; */

/*     if(SendSlotNumber == (RxSlotData[1] - '0')) */
/*     { */
/*         SysProperties.slotInsert[RxSlotData[1] - '0'] = TRUE; */
/*         DoIncSlotIdStep(SendSlotNumber); */
/*     } */
/* } */

/*********************************************************************
*       doAnsTemperature
*       온도 요청에 대한 응답, 회신 되는 응답은 flot임.
**********************************************************************/
/* void DoAnsTemperature(void) */
/* { */
/*     uint8_t i; */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(i = 0; i < 32; i++) */
/*         { */
/*             HAL_Delay(1); */
/*             TestData.temperature[readSlotNumber][i].UI8[0] = RxSlotData[i * 4 + 3]; */
/*             TestData.temperature[readSlotNumber][i].UI8[1] = RxSlotData[i * 4 + 4]; */
/*             TestData.temperature[readSlotNumber][i].UI8[2] = RxSlotData[i * 4 + 5]; */
/*             TestData.temperature[readSlotNumber][i].UI8[3] = RxSlotData[i * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     DoIncSlotIdStep(readSlotNumber); */
/* } */

/* void DoAnsTemperatureState(void) */
/* { */
/*     uint8_t count = 0; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     for(count = 0; count < 32; count++) */
/*     { */
/*         TestData.sensorState[readSlotNumber][count] = (LED_DIPLAY_MODE)RxSlotData[count + 3]; */
/*     } */
/* } */

/* void DoAnsThresholdReq(void) */
/* { */
/*     uni2Byte        crc; */
/*     uint8_t         thresholdData[130]; */

/*     noReturnSendCt = 0; */
/*     thresholdData[0] = RxSlotData[1] - '0'; */
/*     readSlotNumber = thresholdData[0]; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for (int inc = 0; inc < 32; inc++) */
/*         { */
/*             TestData.threshold[readSlotNumber][inc].UI8[0] = RxSlotData[inc * 4 + 3]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[1] = RxSlotData[inc * 4 + 4]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[2] = RxSlotData[inc * 4 + 5]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[3] = RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     if(startThreshold != TRUE)      //초기화 하는 동안은 486 전송 하지 않는다, 초기화 중일때 startThreshold == TRUE 임. */
/*     { */
/*         memcpy(&thresholdData[1], &TestData.threshold[thresholdData[0]][0].UI8[0], 128); */
/*         doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP, OP_WARNING_TEMP_REQ, &thresholdData[0], 129, 132, 152); */
/*         SendUart485String(tx485DataDMA, 152); */
/*     } */

/*     if(startThreshold == TRUE) */
/*     { */
/*         DoIncSlotIdStep(thresholdData[0]); */
/*     } */
/* } */

/* void DoAnsThresholdSet(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         thresholdData[130]; */

/*     noReturnSendCt = 0; */
/*     thresholdData[0] = RxSlotData[1] - '0'; */
/*     readSlotNumber = thresholdData[0]; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             TestData.threshold[readSlotNumber][inc].UI8[0] = RxSlotData[inc * 4 + 3]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[1] = RxSlotData[inc * 4 + 4]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[2] = RxSlotData[inc * 4 + 5]; */
/*             TestData.threshold[readSlotNumber][inc].UI8[3] = RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     memcpy(&thresholdData[1], &TestData.threshold[readSlotNumber][0].UI8[0], 128); */
/*     doMakeSend485Data(tx485DataDMA, CMD_WARNING_TEMP, OP_WARNING_TEMP_SET, &thresholdData[0], 129, 132, 152); */
/*     /\* DBG_LOG("%s: ", __func__); *\/ */
/*     /\* print_bytes(tx485DataDMA, 152); *\/ */
/*     SendUart485String(tx485DataDMA, 152); */
/* } */

/* void DoAnsCalibrationNTCTableCal(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         calData[130]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             osDelay(1); */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[0] = RxSlotData[inc * 4 + 3]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[1] = RxSlotData[inc * 4 + 4]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[2] = RxSlotData[inc * 4 + 5]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[3] = RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     calData[0] = readSlotNumber; */
/*     memcpy((void*)&calData[1],(void*)&TestData.ntcCalibrationTable[readSlotNumber][0].UI8[0], 128); */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL, calData, 129, 132, 152); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_CAL, calData, 129, 132, 152); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 152); *\/ */
/* } */

/* void DoAnsRevisionApplySet(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[2]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionApply[readSlotNumber] = RxSlotData[3]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionApply[readSlotNumber]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_APPLY_SET, revData, 2, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_APPLY_SET, revData, 2, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionConstantSet(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[5]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionConstant[readSlotNumber].UI8[0] = RxSlotData[3]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[1] = RxSlotData[4]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[2] = RxSlotData[5]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[3] = RxSlotData[6]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionConstant[readSlotNumber].UI8[0]; */
/*     revData[2] = TestData.revisionConstant[readSlotNumber].UI8[1]; */
/*     revData[3] = TestData.revisionConstant[readSlotNumber].UI8[2]; */
/*     revData[4] = TestData.revisionConstant[readSlotNumber].UI8[3]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_SET, revData, 6, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_CONSTANT_SET, revData, 6, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionApplyReq(void) */
/* { */

/*     uni2Byte crc; */
/*     uint8_t  revData[2]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionApply[readSlotNumber] = RxSlotData[3]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionApply[readSlotNumber]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_APPLY_REQ, revData, 2, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_APPLY_REQ, revData, 2, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsRevisionConstantReq(void) */
/* { */
/*     uni2Byte crc; */
/*     uint8_t  revData[5]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.revisionConstant[readSlotNumber].UI8[0] = RxSlotData[3]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[1] = RxSlotData[4]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[2] = RxSlotData[5]; */
/*         TestData.revisionConstant[readSlotNumber].UI8[3] = RxSlotData[6]; */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     revData[0] = readSlotNumber; */
/*     revData[1] = TestData.revisionConstant[readSlotNumber].UI8[0]; */
/*     revData[2] = TestData.revisionConstant[readSlotNumber].UI8[1]; */
/*     revData[3] = TestData.revisionConstant[readSlotNumber].UI8[2]; */
/*     revData[4] = TestData.revisionConstant[readSlotNumber].UI8[3]; */

/*     send_external_response(CMD_REVISION, OP_REVISION_CONSTANT_REQ, revData, 6, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_REVISION, OP_REVISION_CONSTANT_REQ, revData, 6, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsCalibrationNTCConstantSet(void) */
/* { */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.ntcCalibrationConst.UI8[0] = RxSlotData[3]; */
/*         TestData.ntcCalibrationConst.UI8[1] = RxSlotData[4]; */
/*         TestData.ntcCalibrationConst.UI8[2] = RxSlotData[5]; */
/*         TestData.ntcCalibrationConst.UI8[3] = RxSlotData[6]; */

/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET, */
/*                            &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */

/* void DoAnsCalibrationNTCTableReq(void) */
/* { */
/*     uint8_t         inc; */
/*     uni2Byte        crc; */
/*     uint8_t         calData[130]; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 130); */

/*     if((crc.UI8[0] == RxSlotData[131]) && (crc.UI8[1] == RxSlotData[132])) */
/*     { */
/*         for(inc = 0; inc < 32; inc++) */
/*         { */
/*             osDelay(1); */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[0] = RxSlotData[inc * 4 + 3]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[1] = RxSlotData[inc * 4 + 4]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[2] = RxSlotData[inc * 4 + 5]; */
/*             TestData.ntcCalibrationTable[readSlotNumber][inc].UI8[3] = RxSlotData[inc * 4 + 6]; */
/*         } */
/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     calData[0] = readSlotNumber; */
/*     memcpy((void*)&calData[1], (void*)&TestData.ntcCalibrationTable[readSlotNumber][0].UI8[0], 128); */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ, calData, 129, 132, 152); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_NTC_CON_TABLE_REQ, calData, 129, 132, 152); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 152); *\/ */
/* } */

/* void DoAnsCalibrationNTCConstantReq(void) */
/* { */
/*     uni2Byte crc; */

/*     noReturnSendCt = 0; */
/*     readSlotNumber = RxSlotData[1] - '0'; */

/*     crc.UI16 = CRC16_Make(&RxSlotData[1], 8); */

/*     if((crc.UI8[0] == RxSlotData[9]) && (crc.UI8[1] == RxSlotData[10])) */
/*     { */
/*         TestData.ntcCalibrationConst.UI8[0] = RxSlotData[3]; */
/*         TestData.ntcCalibrationConst.UI8[1] = RxSlotData[4]; */
/*         TestData.ntcCalibrationConst.UI8[2] = RxSlotData[5]; */
/*         TestData.ntcCalibrationConst.UI8[3] = RxSlotData[6]; */

/*         crcErrorCount = 0; */
/*     } */
/*     else */
/*     { */
/*         crcErrorCount++; */
/*     } */

/*     send_external_response(CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ, &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32); */
/*     /\* doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_NTC_CONSTANT_REQ, &TestData.ntcCalibrationConst.UI8[0], 4, 12, 32); *\/ */
/*     /\* SendUart485String(tx485DataDMA, 32); *\/ */
/* } */
