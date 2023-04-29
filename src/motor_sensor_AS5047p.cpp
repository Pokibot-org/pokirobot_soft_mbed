/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_sensor_AS5047p.h"

namespace sixtron {

    void MotorSensorAS5047P::initSensor() {
        // configure SPI
        _spiAS5047p->format(8, 1);
        _spiAS5047p->frequency(1000000);

        // configure GPIO SPI selection
        _as5047p_spi_cs.write(1);
    }

    uint16_t MotorSensorAS5047P::getSensorValue() {
        _as5047p_spi_cs.write(0);
        uint8_t receivedDataH = (0x3F & _spiAS5047p->write(0xFF)); // Get the first part (8bits)
        uint8_t receivedDataL = _spiAS5047p->write(0xFF); // Get the second part (8bits)
        _as5047p_spi_cs.write(1);
        return (uint16_t)((receivedDataH << 8)
                | (receivedDataL & 0xff)); // Combine the two parts to get a 16bits
    }

} // namespace sixtron
