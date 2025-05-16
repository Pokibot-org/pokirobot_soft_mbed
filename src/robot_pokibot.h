/*
 * POKIBOT 2024
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef ROBOT_POKIBOT_H
#define ROBOT_POKIBOT_H

#include "RBDC.h"
#include "common.h"
#include "lidar_serial.h"
#include "mobile-base_pokibot.h"
#include "motor_sensor_AS5047p.h"
#include "odometry_pokibot.h"

// Robot and control parameters
#define ONE_DEGREE_IN_RAD 0.017453f // 1°
#define DEG_TO_RAD(x) (x * ONE_DEGREE_IN_RAD)

#define LINEAR_PRECISION 0.05f // 2 cm
#define ANGULAR_PRECISION DEG_TO_RAD(5.0f)

#define MAX_MOTOR_PWM 0.85f // With MBED, pwm command between -1.0f and +1.0f max !
#define MOTOR_REDUCTION 50

#define ENC_RESOLUTION 16384
#define ENC_WHEEL_RADIUS 0.0367f // (0.072f / 2.0f) 0.0367 best value with green banebots wheels
#define ENC_WHEELS_DISTANCE (0.328f) // best value with green banebots wheels

void robot_goto(float x,
        float y,
        float theta,
        bool blocking = true,
        sixtron::RBDC_reference reference = sixtron::RBDC_reference::absolute);

void robot_goto(float x,
        float y,
        bool blocking = true,
        sixtron::RBDC_reference reference = sixtron::RBDC_reference::absolute);

void robot_vector(float x, float y);

void start_robot_pokibot_control_thread();

void set_ignore_lidar(bool state);

void robot_normal_speed();
void robot_low_speed();
void robot_high_speed();

#endif // ROBOT_POKIBOT_H
