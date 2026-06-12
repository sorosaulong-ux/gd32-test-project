#ifndef __BLE_H
#define __BLE_H

#include "gd32a7xx.h"
#include <stdint.h>

void ble_init(void);
uint8_t ble_is_connected(void);

#endif
