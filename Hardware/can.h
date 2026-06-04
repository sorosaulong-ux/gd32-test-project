#ifndef __CAN_H
#define __CAN_H

#include "gd32a7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

void      can_gpio_config(void);
void      can_config(can_dtm_canx_enum dtm_canx, uint32_t baudrate_khz);
void      can_print_error_status(can_dtm_canx_enum dtm_canx);

ErrStatus can_send_std_frame(can_dtm_canx_enum dtm_canx,
                              uint32_t std_id,
                              const uint8_t *data, uint8_t len);
ErrStatus can_recv_frame_poll(can_dtm_canx_enum dtm_canx,
                               can_receive_message_struct *rx_msg);
void      can_flush_rx_fifo0(can_dtm_canx_enum dtm_canx);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */