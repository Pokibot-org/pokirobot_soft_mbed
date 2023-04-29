/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "odometry_pokibot.h"

namespace sixtron {

    void OdometryPokibot::init() {

        _left->init();
        _right->init();
    }

    void OdometryPokibot::update() {

        _left->update();
        _right->update();

        compute(_left->getTickCount(), _right->getTickCount());
    }
}
