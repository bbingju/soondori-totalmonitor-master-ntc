#include "system_task.h"
#include "0_GlobalValue.h"
#include "0_UartCallback.h"
#include "protocol.h"
#include "external_uart_task.h"
#include "internal_uart_task.h"

#include <math.h>
#include <string.h>

/*********************************************************************
 *       Private variables
 **********************************************************************/
// UPDATA_FLAG     Updata_Flag;

GPIO_TypeDef *SLAVE_CS_PORT[4] = {
    SLAVE_CS0_GPIO_Port, SLAVE_CS1_GPIO_Port, SLAVE_CS2_GPIO_Port,
    SLAVE_CS3_GPIO_Port}; // todo : 순서를 0123 으로 바꿔야함
uint16_t SLAVE_CS_PIN[4] = {
    SLAVE_CS0_Pin, SLAVE_CS1_Pin, SLAVE_CS2_Pin,
    SLAVE_CS3_Pin}; // back plate 의 컨넥터가 잘못 되어 있음

uint8_t TxDataBuffer[UART_TX_BUF_MAX] = {0}; // dma 용도
/* uint8_t RxDataDMA[134 * 10] = {0};           // dma 용도 */

/* uint8_t         RxSlotData[140]  = { 0 };                       //parsing
 * 용도 */
uint8_t RxSlotDataLength = 0;
uint8_t RxReadCount = 0;

uint8_t ReceveDataLength = 0;
uint8_t SendSlotNumber = 0;
uint8_t SendChannel = 0;
uint8_t SlotCheck = FALSE;
uint8_t noReturnSendCt = 0;

uni4Byte readTemp;
uint8_t readSlotNumber;

uint8_t temp;
uint8_t crcErrorCount = 0;

uint8_t tempReqFlag = FALSE;
uint8_t semCount = 0;
uint8_t startThreshold = FALSE;
/* RX_QUEUE_STRUCT RxQueue; */

extern int int_tx_completed;
extern int int_rx_completed;
extern IWDG_HandleTypeDef hiwdg;

void check_slots_inserted(struct slot_properties_s *slots, int num_of_slots)
{
    static uint8_t buf[164] = {0};

    for (int i = 0; i < num_of_slots; i++) {
        for (int j = 0; j < 11; j++) {
            HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin,
                              GPIO_PIN_RESET);

            doMakeSendSlotData(buf, slots[i].id + 0x30, CMD_TEMP_REQ, buf, 0,
                               SEND_DATA_LENGTH);
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

    	osDelay(100);

        slots[i].inserted = noReturnSendCt > 9 ? false : true;
        noReturnSendCt = 0;
    }

    for (int i = 0; i < num_of_slots; i++) {
        DBG_LOG("slot %d inserted %s\n", slots[i].id,
                slots[i].inserted ? "TRUE" : "FALSE");
    }
}

void system_task(void const *argument)
{
	SysProperties.InterfaceStep = STEP_SLOT_ID;
	/* SysProperties.InterfaceStep = STEP_READ_THRESHOLD; */
	/* SysProperties.InterfaceStep = STEP_TEMP_READ; */
	/* SysProperties.slots[0].inserted = true; */
	/* SysProperties.slots[2].inserted = true; */
	/* SysProperties.slots[3].inserted = true; */

	HAL_GPIO_WritePin(SLAVE_OE_GPIO_Port, SLAVE_OE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLAVE_OE1_GPIO_Port, SLAVE_OE1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_EN_BT_GPIO_Port, UART_EN_BT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_EN_SLOT_GPIO_Port, UART_EN_SLOT_Pin, GPIO_PIN_SET);

	DoSlotReset(ALL_SLOT);
	DoRejectSlot();
	HAL_IWDG_Refresh(&hiwdg);
	osDelay(100);
	HAL_IWDG_Refresh(&hiwdg);
	// todo : 센서 타입을 슬레이브 보드에서 읽어 와야 함.

	for (;;) {
		switch (SysProperties.InterfaceStep) {

		case STEP_SLOT_ID: { // 부팅 하면 각 슬롯의 id 를 지정 한다. id 는
			volatile bool existed_any_slot = false;

			while (!existed_any_slot) {
				/* check_slots_inserted(SysProperties.slots, MAX_SLOT_NUM); */

				for (int i = 0; i < MAX_SLOT_NUM; i++) {
					send_slot_id_req(i);
					osDelay(500);
				}

				osDelay(1000);

				for (int i = 0; i < MAX_SLOT_NUM; i++) {
					struct slot_properties_s *slot = &SysProperties.slots[i];
					if (slot->inserted) {
						existed_any_slot = true;
						break;
					}
				}

			}
			/* if (system_reset_needed) { */
			/*     HAL_IWDG_Refresh(&hiwdg); */
			/*     osDelay(500); */
			/*     HAL_NVIC_SystemReset(); */
			/* } */

			SysProperties.InterfaceStep = STEP_READ_THRESHOLD;
			break;
		}

		case STEP_READ_THRESHOLD: //각 슬롯의 경고 온도 값을 불러 온다.
			startThreshold = TRUE;
			for (int i = 0; i < MAX_SLOT_NUM; i++) {
				struct slot_properties_s *slot = &SysProperties.slots[i];
				DoThresholdReq(slot);
			}
			osDelay(100);
			SysProperties.InterfaceStep = STEP_TEMP_READ;
			break;

		case STEP_TEMP_READ: // 각 슬롯의 id 설정 완료 후 온도센서의 온도를 요청한다.
			for (int i = 0; i < MAX_SLOT_NUM; i++) {
				struct slot_properties_s *slot = &SysProperties.slots[i];
				/* DBG_LOG("slot->id: %d, type: %d, inserted: %d\n",
				   slot->id, slot->type, slot->inserted); */
				DoReqTemperature(slot);
				osDelay(SysProperties.interval_ms / 4);
				DoReqTemperatureState(slot);
				osDelay(SysProperties.interval_ms / 4);
			}
			/* SysProperties.InterfaceStep = STEP_READ_THRESHOLD; */
			if (noReturnSendCt > 10)
				noReturnSendCt = 0;
			break;
		}
	}
}
