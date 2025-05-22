/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "mobile-base_pokibot.h"

namespace sixtron {

    void MotorBasePokibot::init() {

        terminal_debug("pid_motor_params.dt_seconds = %f\n", _dt_pid);

        // anciennemment PH_1 et PB_7, inversé en 2024 pour avoir le robot dans le bon sens
        _motorLeft = new sixtron::MotorDCPokibot(_dt_pid,
                _sensorLeft,
                PH_1,
                PB_7,
                _motor_pid_params,
                _max_motor_pwm,
                MOTOR_DIR_NORMAL);
        // anciennement PA_4 et PB_6, inversé en 2024 pour avoir le robot dans le bon sens
        _motorRight = new sixtron::MotorDCPokibot(_dt_pid,
                _sensorRight,
                PA_4,
                PB_6,
                _motor_pid_params,
                _max_motor_pwm,
                MOTOR_DIR_NORMAL);

        _motorLeft->init();
        _motorRight->init();
        _motorLeft->start();
        _motorRight->start();
    }

    void MotorBasePokibot::update() {

        computeMotorSpeeds();

        _motorLeft->setSpeed(_targetSpeedMotorLeft);
        _motorRight->setSpeed(_targetSpeedMotorRight);

        if ((_targetSpeedMotorLeft > 0.0f && _targetSpeedMotorRight < 0.0f)
                || (_targetSpeedMotorLeft < 0.0f && _targetSpeedMotorRight > 0.0f)) {
            _running_side = TURNING_ON_ITSLEF;
        } else if (_targetSpeedMotorLeft >= 0.0f && _targetSpeedMotorRight >= 0.0f) {
            _running_side = RUNNING_FRONT;
        } else if (_targetSpeedMotorLeft < 0.0f && _targetSpeedMotorRight < 0.0f) {
            _running_side = RUNNING_BACK;
        } else {
            _running_side = NOT_MOVING;
        }

        if (_mobile_base_status == mobile_base_start) {
            _motorLeft->update();
            _motorRight->update();
        } else if (_mobile_base_status == mobile_base_stop) {
            _motorLeft->standby();
            _motorRight->standby();
        }


    }

    int MotorBasePokibot::get_running_side() {
        return _running_side;
    }

    MotorDCPokibot *MotorBasePokibot::getMotorLeft() {
        return _motorLeft;
    }

    MotorDCPokibot *MotorBasePokibot::getMotorRight() {
        return _motorRight;
    }
}
