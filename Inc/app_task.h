#ifndef APP_TASK_H
#define APP_TASK_H

#include "cmsis_os.h"
#include "0_Util.h"

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

//  공통
void SlotRxFunction(void);
void UnpackingRxQueue(void);
void DoSlotReset(uint8_t slot);
void DoSelectSlot(uint8_t slot);
void DoRejectSlot(void);

//  Tx
void app_task(void const *argument);

#endif /* APP_TASK_H */

