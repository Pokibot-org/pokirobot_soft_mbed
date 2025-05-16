/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef MOBILE_BASE_POKIBOT_H
#define MOBILE_BASE_POKIBOT_H

#include "common.h"
#include "mbed.h"
#include "mobile_base/mobile_base_differential.h"
#include "motor_DC_pokibot.h"
#include "motor_sensor_AS5047p.h"

namespace sixtron {

#define NOT_MOVING 0
#define RUNNING_FRONT 1
#define RUNNING_BACK 2
#define TURNING_ON_ITSLEF 3

    class MotorBasePokibot: public MobileBaseDifferential {

    public:
        MotorBasePokibot(float rate_dt,
                MotorSensorEncoder *sensor_left,
                MotorSensorEncoder *sensor_right,
                PID_params motor_pid_params,
                float entraxe,
                float max_motor_pwm):
                MobileBaseDifferential(entraxe),
                _dt_pid(rate_dt),
                _motor_pid_params(motor_pid_params),
                _max_motor_pwm(max_motor_pwm),
                _sensorLeft(sensor_left),
                _sensorRight(sensor_right) {};

        ~MotorBasePokibot() = default;

        // Specific for Pokibot, uses two PIDs to control each motor.
        void init() override;

        // Specific for Pokibot.
        void update() override;

        int get_running_side();

        MotorDCPokibot *getMotorLeft();
        MotorDCPokibot *getMotorRight();

    private:
        float _dt_pid; // in [s]
        PID_params _motor_pid_params;
        float _max_motor_pwm;

        int _running_side;

        MotorSensorEncoder *_sensorLeft;
        MotorSensorEncoder *_sensorRight;

        MotorDCPokibot *_motorLeft;
        MotorDCPokibot *_motorRight;
    };
}

#endif // MOBILE_BASE_POKIBOT_H
