/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef ODOMETRY_POKIBOT_H
#define ODOMETRY_POKIBOT_H

#include "mbed.h"
#include "motor_sensor_AS5047p.h"
#include "odometry/odometry_two_encoders.h"

namespace sixtron {

#define ENC_LEFT 0
#define ENC_RIGHT 1

    class OdometryPokibot: public OdometryTwoEncoders {

    public:
        OdometryPokibot(float rate_hz, MotorSensorEncoder *left, MotorSensorEncoder *right, float motor_resolution, float enc_wheel_radius, float entraxe):
                OdometryTwoEncoders(rate_hz,
                        motor_resolution,
                        enc_wheel_radius,
                        entraxe),
                _left(left),
                _right(right) {};

        ~OdometryPokibot() = default;

        // Specific for Pokibot, uses SPI.
        void init() override;

        // Specific for Pokibot , uses SPI to get encoders values.
        void update() override;

    private:
        MotorSensorEncoder *_left;
        MotorSensorEncoder *_right;
    };
}

#endif // ODOMETRY_POKIBOT_H
