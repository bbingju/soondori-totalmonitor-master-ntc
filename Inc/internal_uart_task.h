#ifndef INTERNAL_UART_TASK_H
#define INTERNAL_UART_TASK_H

#include <stdint.h>

struct internal_uart_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

    void send_slot_id_req(uint8_t id);
    int send_internal_req(uint8_t id, uint8_t cmd, void *data, uint16_t length);
    int push_internal_resp(void *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_UART_TASK_H */
