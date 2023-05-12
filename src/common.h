/*
* POKIBOT 2023
* Mbed Software for Pokirobot V1
* SPDX-License-Identifier: AGPL-3.0-or-later
*/

#ifndef POKIROBOT_SOFT_MBED_COMMON_H
#define POKIROBOT_SOFT_MBED_COMMON_H

#include "mbed.h"

// LIDAR
#define LIDAR_THREAD_FLAG 0x03
EventFlags lidarThreadFlag;
uint8_t lidar_new_value, lidar_processing;
volatile uint16_t lidar_overflow = 0, start_sequence_incr = 0;

#endif // POKIROBOT_SOFT_MBED_COMMON_H
