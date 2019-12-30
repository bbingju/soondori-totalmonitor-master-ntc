#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

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

#endif /* SYSTEM_TASK_H */

