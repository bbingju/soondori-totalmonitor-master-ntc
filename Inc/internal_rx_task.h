#ifndef INTERNAL_RX_TASK_H
#define INTERNAL_RX_TASK_H

struct internal_rx_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

    int internal_rx_msg_push(void *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_RX_TASK_H */
