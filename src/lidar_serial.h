/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H
#define POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H

#include "common.h"
// #include <stm32f4xx_ll_exti.h>

#define LDS_01_TRAM_LENGTH 2518

extern uint8_t lidar_trame[LDS_01_TRAM_LENGTH];

void init_lidar_serial();

#endif // POKIROBOT_SOFT_MBED_LIDAR_SERIAL_H