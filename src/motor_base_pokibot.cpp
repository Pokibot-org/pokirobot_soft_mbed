/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_base_pokibot.h"

namespace sixtron {

    void MotorBasePokibot::init() {

        sixtron::PID_params pid_motor_params;
        pid_motor_params.Kp = 15.0f;
        pid_motor_params.Ki = 40.0f; // 5.0
        pid_motor_params.Kd = 0.00f;
        pid_motor_params.dt_seconds = _dt_pid;
        pid_motor_params.ramp = 1.0f * _dt_pid;

        _motorLeft = new sixtron::MotorDCPokibot(
                _dt_pid, _sensorLeft, PH_1, PB_7, pid_motor_params, MAX_MOTOR_PWM);
        _motorRight = new sixtron::MotorDCPokibot(
                _dt_pid, _sensorRight, PA_4, PB_6, pid_motor_params, MAX_MOTOR_PWM);

        _motorLeft->init();
        _motorRight->init();
        _motorLeft->start();
        _motorRight->start();
    }

    void MotorBasePokibot::update() {

        computeMotorSpeeds();

        _motorLeft->setSpeed(_targetSpeedMotorLeft);
        _motorRight->setSpeed(_targetSpeedMotorRight);

        _motorLeft->update();
        _motorRight->update();
    }
}
