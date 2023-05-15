/*
* POKIBOT 2023
* Mbed Software for Pokirobot V1
* SPDX-License-Identifier: AGPL-3.0-or-later
*/

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef COMMON_POKIBOT_H
#define COMMON_POKIBOT_H

#define POKI_SO PA_14       // U10 pin 7
#define POKI_DO PA_13       // U10 pin 8
#define POKI_S1 PH_1        // U11 pin 7
#define POKI_D1 PA_4        // U11 pin 8
#define POKI_S2 PC_1        // U12 pin 7
#define POKI_D2 PB_0        // U12 pin 8
#define POKI_S3 PC_3        // U20 pin 7
#define POKI_D3 PC_0        // U20 pin 8
#define TMC_RX0 PD_2        // U10 -> U20 pin 4
#define TMC_TX0 PC_12       // U10 -> U20 pin 4

#define PWM_SERVO0 PB_10    // J20
#define PWM_SERVO1 PB_9     // J21
#define PWM_SERVO2 PB_8     // J22

#define POMPE PB_3          // J23
#define VANNE PB_1          // J24
#define ELECTROAIMANT PC_4  // J25

#define SB_TX PA_1          // J26
#define SB_RX PA_0          // J26
#define SB_RST PC_2         // J26

#define SEG7_RX PC_10      // J38
#define SEG7_TX PC_11      // J38

#define SW_01 PB_13         // J30
#define SW_02 PB_14         // J31
#define SW_03 PB_15         // J32
#define SW_04 PB_12         // J33
#define SW_05 PA_11         // J34
#define SW_06 PA_12         // J35

#define LIDAR_TX PA_10      // J36
#define LIDAR_RX PA_9       // J36

#define LORA_RX PC_7        // J37 (J39 BT)
#define LORA_TX PC_6        // J37 (J39 BT)

#define SW_ROBOT PC_13      // J40
#define SW_COTE PA_15       // J41
#define TIRETTE PC_9        // J42

#define LED_RED PB_4
#define LED_GREEN PC_8
#define LED_BLUE PB_5

#define ENC_MOSI PA_7
#define ENC_MISO PA_6
#define ENC_SCK  PA_5
#define ENC_CS_LEFT PA_8
#define ENC_CS_RIGHT PB_2


#include "mbed.h"
extern void terminal_printf(const char *fmt, ...);

#endif