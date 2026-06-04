#ifndef _CAN_H
#define _CAN_H
#include "main.h"

typedef struct {
     
  uint32_t id;
  uint8_t data;
  uint8_t len;

} can_hand_t;
/*
can_hand_t dtm_can0_data;
can_hand_t dtm_can2_data;
can_hand_t dtm_can5_data;
can_hand_t dtm_can4_data;
*/
void can_gpio_config(void);
void bsp_board_config(void);
void can_config(can_dtm_canx_enum dtm_canx, uint32_t baudrate_khz);
void canfd_config(void);
void communication_check(void);
ErrStatus dtm_can4_message_transmit_test(void);
ErrStatus dtm_can0_message_transmit_test(void);

void DTM_CAN0_INT0_IRQHandler(void);
void DTM_CAN0_INT2_IRQHandler(void);
void DTM_CAN0_INT4_IRQHandler(void);
void DTM_CAN0_INT5_IRQHandler(void);
void can_enable_rx_interrupt(can_dtm_canx_enum dtm_canx, uint8_t irqn_priority);
ErrStatus can_send_std_frame(can_dtm_canx_enum dtm_canx, uint32_t std_id, uint8_t *data, uint8_t len);

#endif
