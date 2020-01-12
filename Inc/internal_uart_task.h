#ifndef INTERNAL_UART_TASK_H
#define INTERNAL_UART_TASK_H

/* #include "0_GlobalValue.h" */
#include "app_ctx.h"
#include "frame.h"
#include <stdint.h>

struct internal_uart_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

void response_from_internal(struct internal_frame *);
void internal_rx_task(void const *arg);
void internal_rx_notify();


void request_to_internal__SLOT_ID_REQ(struct slot_s *);
void request_to_internal__TEMPERATURE_REQ(struct slot_s *);
void request_to_internal__TEMPERATURE_STATE_REQ(struct slot_s *);
void request_to_internal__THRESHOLD_SET(struct slot_s *, uint8_t, float);
void request_to_internal__THRESHOLD_REQ(struct slot_s *);
void request_to_internal__REVISION_APPLY_SET(struct slot_s *, uint8_t const);
void request_to_internal__REVISION_APPLY_REQ(struct slot_s *);
void request_to_internal__REVISION_CONST_SET(struct slot_s *);
void request_to_internal__REVISION_CONST_REQ(struct slot_s *);
void request_to_internal__REVISION_TR_CONST_SET(struct slot_s *, float r1, float r2);
void request_to_internal__REVISION_TR_CONST_REQ(struct slot_s *);
void request_to_internal__CALIBRATION_NTC_TABLE_CAL(struct slot_s *);
void request_to_internal__CALIBRATION_NTC_TABLE_REQ(struct slot_s *);
void request_to_internal__CALIBRATION_NTC_CONST_SET(struct slot_s *);
void request_to_internal__CALIBRATION_NTC_CONST_REQ(struct slot_s *);

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_UART_TASK_H */
