/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H
#define POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H

#include "mbed.h"
// #include <stm32f4xx_ll_exti.h>

// LIDAR Thread
#define LIDAR_THREAD_FLAG 0x03
extern EventFlags lidarThreadFlag;
extern uint8_t lidar_new_value, lidar_processing;
extern uint16_t lidar_overflow, start_sequence_incr;

// LIDAR Parameters
#define LDS_01_START_FIRST 0xFA
#define LDS_01_START_SECOND 0xA0
#define LDS_01_TRAM_LENGTH 2520
extern uint8_t lidar_frame[LDS_01_TRAM_LENGTH];

void init_lidar_serial();

#endif // POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H
