/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_DC_pokibot.h"

namespace sixtron {

    void MotorDCPokibot::initHardware() {
        // Init Motor PWM
        _dir.write(0);
        _pwm.period_us(50); // 20kHz
        _pwm.write(0.0f);
    }

    float MotorDCPokibot::getSensorSpeed() {
        return _sensor->getSpeed();
    }

    void MotorDCPokibot::setPWM(float pwm) {
        // update hardware motor PWM
        if (pwm >= 0.0f) {
            _dir.write(0);
        } else {
            pwm = -pwm;
            _dir.write(1);
        }

        _pwm.write(pwm);
    }

} // namespace sixtron
