/*!
 *  \file    Common.h
 *  \brief   Shared type definitions for MQTT library — GD32A7 port
 *
 *  Use standard C99 types (not custom short aliases) to avoid conflicts
 *  with <stdint.h> used everywhere in GD32A7.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdbool.h>

/* Type aliases used by MqttKit (same names as original GD32F3 code) */
typedef bool                uint1;
typedef uint8_t             uint8;
typedef int8_t              int8;
typedef uint16_t            uint16;
typedef int16_t             int16;
typedef uint32_t            uint32;
typedef int32_t             int32;

#endif /* __COMMON_H__ */
