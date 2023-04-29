/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef MOTOR_BASE_POKIBOT_H
#define MOTOR_BASE_POKIBOT_H

#include "mbed.h"
#include "motor_DC_pokibot.h"
#include "motor_base/motor_base_two_wheels.h"
#include "motor_sensor_AS5047p.h"

namespace sixtron {

#define ENC_WHEELS_DISTANCE (0.315f)
#define MAX_MOTOR_PWM 0.4f // With MBED, pwm command between -1.0f and +1.0f max !

    class MotorBasePokibot: public MotorBaseTwoWheels {

    public:
        MotorBasePokibot(
                float rate_dt, MotorSensorEncoder *sensor_left, MotorSensorEncoder *sensor_right):
                MotorBaseTwoWheels(ENC_WHEELS_DISTANCE),
                _dt_pid(rate_dt),
                _sensorLeft(sensor_left),
                _sensorRight(sensor_right) {};

        ~MotorBasePokibot() = default;

        // Specific for Pokibot, uses two PIDs to control each motor.
        void init() override;

        // Specific for Pokibot.
        void update() override;

    private:
        float _dt_pid; // in [s]

        MotorSensorEncoder *_sensorLeft;
        MotorSensorEncoder *_sensorRight;

        MotorDCPokibot *_motorLeft;
        MotorDCPokibot *_motorRight;
    };
}

#endif // MOTOR_BASE_POKIBOT_H
