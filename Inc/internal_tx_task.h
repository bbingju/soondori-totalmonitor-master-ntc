#ifndef INTERNAL_TX_TASK_H
#define INTERNAL_TX_TASK_H

struct internal_tx_msg_s;

#ifdef __cplusplus
extern "C" {
#endif

    int internal_tx_msg_push(uint8_t id, uint8_t cmd, void *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_TX_TASK_H */
