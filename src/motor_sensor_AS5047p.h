/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H
#define CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H

#include "mbed.h"
#include "motor_sensor_encoder.h"

namespace sixtron {

#define DIR_NORMAL (+1)
#define DIR_INVERTED (-1)

    class MotorSensorAS5047P: public MotorSensorEncoder {
    public:
        MotorSensorAS5047P(SPI *spiAS5047p,
                PinName sensor_spi_select,
                float rate_dt,
                int32_t sensorResolution,
                float motorResolution,
                float motorWheelRadius,
                int encDirection = DIR_NORMAL):
                MotorSensorEncoder(
                        rate_dt, sensorResolution, motorResolution, motorWheelRadius, encDirection),
                _spiAS5047p(spiAS5047p),
                _as5047p_spi_cs(sensor_spi_select) {};

    protected:
        void initSensor() override;
        uint16_t getSensorValue() override;

    private:
        SPI *_spiAS5047p;
        DigitalOut _as5047p_spi_cs;
    };

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H
