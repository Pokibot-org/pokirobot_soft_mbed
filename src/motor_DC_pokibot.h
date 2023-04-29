/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef CATIE_SIXTRON_MOTOR_DC_POKIBOT_H
#define CATIE_SIXTRON_MOTOR_DC_POKIBOT_H

#include "mbed.h"
#include "motor/motor_DC.h"
#include "motor_sensor_AS5047p.h"

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

    class MotorDCPokibot: public MotorDC {

    public:
        MotorDCPokibot(float rate_dt,
                MotorSensor *sensor,
                PinName dir,
                PinName pwm,
                PID_params motor_pid,
                float max_pwm = DEFAULT_MOTOR_MAX_PWM):
                MotorDC(rate_dt, motor_pid, max_pwm), _sensor(sensor), _dir(dir), _pwm(pwm) {};

    private:
        void initHardware() override;
        void setPWM(float pwm) override;
        float getSensorSpeed() override;

    private:
        MotorSensor *_sensor;
        DigitalOut _dir;
        PwmOut _pwm;
    };

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_DC_POKIBOT_H
