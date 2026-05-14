/*
 * POKIBOT 2024
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "robot_pokibot.h"

// CONTROL LOOP (Mbed THREAD in real time)
#define CONTROL_THREAD_RATE 1ms // loop at 1kHz, MANDATORY BECAUSE OF SENSORS !!
#define CONTROL_THREAD_FLAG 0x02
Ticker controlThreadTicker;
EventFlags controlThreadFlag;
Thread controlThread(osPriorityRealtime, OS_STACK_SIZE * 4);

// Lidar
Thread lidarThread(osPriorityAboveNormal, OS_STACK_SIZE);
volatile bool ignore_lidar = false;

SPI spiAS5047p(ENC_MOSI, ENC_MISO, ENC_SCK); // mosi, miso, sclk

sixtron::MotorBasePokibot *basePokibot;
sixtron::MotorSensorAS5047P *sensorLeft;
sixtron::MotorSensorAS5047P *sensorRight;
sixtron::OdometryPokibot *odom;

sixtron::RBDC *rbdc_poki;

sixtron::speed_profile default_angular_speeds;
sixtron::speed_profile default_linear_speeds, high_linear_speeds, low_linear_speeds;

const string rbdc_status[RBDC_MAX_STATUS] = {
    "RBDC_standby",
    "RBDC_working",
    "RBDC_done",
    "RBDC_correct_f_angle",
    "RBDC_moving",
    "RBDC_moving_&_angle",
    "RBDC_correct_i_angle",
};
volatile int rbdc_result = sixtron::RBDC_status::RBDC_standby;

// Be aware this is a blocking function by default
void robot_goto(float x, float y, float theta, bool blocking, sixtron::RBDC_reference reference) {

    rbdc_poki->setTarget(x, y, theta, reference);
    ThisThread::sleep_for(100ms); // le temps que l'asserv s'update, à remplacer par un flag
    if (blocking) {
        while (rbdc_result != sixtron::RBDC_status::RBDC_done) {
            ThisThread::sleep_for(10ms);
        }
    }
}

// Just go to X/Y, no final theta
void robot_goto(float x, float y, bool blocking, sixtron::RBDC_reference reference) {

    rbdc_poki->setTarget(x, y, reference);
    ThisThread::sleep_for(100ms);
    if (blocking) {
        while (rbdc_result != sixtron::RBDC_status::RBDC_done) {
            ThisThread::sleep_for(10ms);
        }
    }
}

void robot_set_position(float x, float y, float theta) {
    rbdc_poki->setAbsolutePosition(x, y, theta);
    ThisThread::sleep_for(200ms);
}

void set_ignore_lidar(bool state) {
    ignore_lidar = state;
}

void checkLidar() {
    if (!ignore_lidar) {
        int current_moving_side = rbdc_poki->getRunningDirection();

        if ((current_moving_side == RBDC_DIR_FORWARD) && lidar_front_trig) {
            rbdc_poki->pause();
        } else if ((current_moving_side == RBDC_DIR_BACKWARD) && lidar_back_trig) {
            rbdc_poki->pause();
        }
    }
}

void robot_normal_speed() {
    rbdc_poki->resetSpeedProfile(sixtron::speed_controller_type::linear);
}

void robot_high_speed() {
    rbdc_poki->setSpeedProfile(sixtron::speed_controller_type::linear, high_linear_speeds);
}

void robot_low_speed() {
    rbdc_poki->setSpeedProfile(sixtron::speed_controller_type::linear, low_linear_speeds);
}

/* ######################  BOUCLE D'ASSERVISSEMENT   ############################################ */

void controlThreadUpdate() {
    controlThreadFlag.set(CONTROL_THREAD_FLAG);
}

void control() {

    terminal_printf("[ASSERV] Init ...\n");

    // Convert current rate of the loop in seconds (float)
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(CONTROL_THREAD_RATE);
    float dt_pid = f_secs.count(); // Very important for all PIDs
    float hz_pid = 1.0f / dt_pid;
    terminal_debug("control frequency = %f\n", hz_pid);

    // create encoders
    sensorLeft = new sixtron::MotorSensorAS5047P(&spiAS5047p,
            ENC_CS_LEFT,
            dt_pid,
            ENC_RESOLUTION,
            ENC_RESOLUTION * MOTOR_REDUCTION,
            ENC_WHEEL_RADIUS,
            DIR_INVERTED);

    sensorRight = new sixtron::MotorSensorAS5047P(&spiAS5047p,
            ENC_CS_RIGHT,
            dt_pid,
            ENC_RESOLUTION,
            ENC_RESOLUTION * MOTOR_REDUCTION,
            ENC_WHEEL_RADIUS,
            DIR_NORMAL);

    // Create odometry. Will be init by RBDC.
    odom = new sixtron::OdometryPokibot(hz_pid,
            sensorLeft,
            sensorRight,
            (ENC_RESOLUTION * MOTOR_REDUCTION),
            ENC_WHEEL_RADIUS,
            ENC_WHEELS_DISTANCE);

    sixtron::PID_params pid_motor_params;
    pid_motor_params.Kp = 6.0f;
    pid_motor_params.Ki = 20.0f; // 5.0
    pid_motor_params.Kd = 0.00f;
    pid_motor_params.Kf = 4.0f;
    pid_motor_params.dt_seconds = dt_pid;
    // pid_motor_params.ramp_high = 2.0f; // acc_max when ramping up (positive or negative) in
    // [m/s²]. pid_motor_params.ramp_low = 4.0f; // acc_max when ramping down (positive or negative)
    // in [m/s²].

    // Create robot base. This will init all motors as well.  Will be init by RBDC.
    basePokibot = new sixtron::MotorBasePokibot(
            dt_pid, sensorLeft, sensorRight, pid_motor_params, ENC_WHEELS_DISTANCE, MAX_MOTOR_PWM);

    // Setup RBDC
    sixtron::RBDC_params rbdc_poki_params;
    rbdc_poki_params.rbdc_format = sixtron::RBDC_format::differential_robot;

    // Set behaviors for linear and angular control loops
    rbdc_poki_params.linear_parameters.movement = sixtron::speed_movement_type::trapezoidal_only;
    rbdc_poki_params.angular_parameters.movement = sixtron::speed_movement_type::trapezoidal_only;

    // Define at least one default speed profile.
    default_linear_speeds.max_accel = 0.7;
    default_linear_speeds.max_decel = 1.6;
    default_linear_speeds.max_speed = 0.8f; // in [m/s], neet at least MAX_MOTOR_PWM to 0.7

    high_linear_speeds.max_accel = 1.2;
    high_linear_speeds.max_decel = 2.0;
    high_linear_speeds.max_speed = 1.0f; // need to "MAX_MOTOR_PWM" to 0.85

    low_linear_speeds.max_accel = 0.3;
    low_linear_speeds.max_decel = 1.2;
    low_linear_speeds.max_speed = 0.4f;
    //
    // default_angular_speeds.max_accel = 0.3;
    // default_angular_speeds.max_decel = 1.2;
    // default_angular_speeds.max_speed = 0.4f;

    // Apply the default speed profile into RBDC parameters
    rbdc_poki_params.linear_parameters.default_speeds = default_linear_speeds;

    // very important to fine tune these two value with the robot behavior
    rbdc_poki_params.linear_parameters.trapeze_tuning.pivot_gain = 0.100f; // See RBDC source code
    rbdc_poki_params.linear_parameters.trapeze_tuning.precision_gain = 0.1f;

    // set fine tune gain on angular just in case ?? not sure
    rbdc_poki_params.angular_parameters.trapeze_tuning.pivot_gain = 0.100f; // See RBDC source code
    rbdc_poki_params.angular_parameters.trapeze_tuning.precision_gain = 0.1f;

    // NOT USED IN PID ONLY MODE
    if (rbdc_poki_params.angular_parameters.movement != sixtron::speed_movement_type::pid_only) {
        rbdc_poki_params.angular_parameters.default_speeds.max_accel = 1.0f * M_PI_F;
        rbdc_poki_params.angular_parameters.default_speeds.max_decel = 4.0f * M_PI_F;
        rbdc_poki_params.angular_parameters.default_speeds.max_speed = 3.0f * M_PI_F; // in [rad/s]
    }

    // Setup precisions
    rbdc_poki_params.linear_parameters.precision = LINEAR_PRECISION;
    rbdc_poki_params.angular_parameters.precision = ANGULAR_PRECISION;

    // Need to go backward
    rbdc_poki_params.can_go_backward = true;
    rbdc_poki_params.dt_seconds = dt_pid;

    /* USE THIS BLOC ONLY IF LINEAR CONTROL MOVEMENT USE THE PID! */
    // if ((rbdc_poki_params.linear_parameters.movement == sixtron::speed_movement_type::pid_only)
    //         || (rbdc_poki_params.linear_parameters.movement
    //                 == sixtron::speed_movement_type::trapezoidal_and_pid)) {
    //     rbdc_poki_params.linear_parameters.pid_params.Kp = 1.0f;
    //     rbdc_poki_params.linear_parameters.pid_params.Ki = 0.001f;
    //     rbdc_poki_params.linear_parameters.pid_params.Kd = 0.0f;
    //     rbdc_poki_params.linear_parameters.pid_params.ramp_high = 0.5f
    //             / rbdc_poki_params.linear_parameters.pid_params.Kp; // Not outputs accel / decel
    //             !!
    //     rbdc_poki_params.linear_parameters.pid_params.ramp_low
    //             = 2.0f / rbdc_poki_params.linear_parameters.pid_params.Kp;
    // }

    /* USE THIS BLOC ONLY IF ANGULAR CONTROL MOVEMENT USE THE PID! */
    if (rbdc_poki_params.angular_parameters.movement == sixtron::speed_movement_type::pid_only) {
        // Theta, or angular speed, PID parameters
        rbdc_poki_params.angular_parameters.pid_params.Kp = 2.0f;
        rbdc_poki_params.angular_parameters.pid_params.Ki = 0.0f;
        rbdc_poki_params.angular_parameters.pid_params.Kd = 0.0f;
        rbdc_poki_params.angular_parameters.pid_params.ramp
                = 20.0f / rbdc_poki_params.angular_parameters.pid_params.Kp;
    }

    rbdc_poki = new sixtron::RBDC(odom,
            basePokibot,
            rbdc_poki_params); // will init odom and robot base as well
    sixtron::position target_pos;
    rbdc_poki->setTarget(0.0f, 0.0f, 0.0f);

    // #endif

    // start control loop ticker
    controlThreadTicker.attach(&controlThreadUpdate, CONTROL_THREAD_RATE);
    terminal_printf("[ASSERV] Init done.\n");
    float time_passed = 0.0f;

    sixtron::target_speeds debug_base;
    debug_base.cmd_lin = 0.20f;
    debug_base.cmd_rot = 0.00f;

    sixtron::target_speeds speeds_nulls;
    speeds_nulls.cmd_lin = 0.00f;
    speeds_nulls.cmd_rot = 0.00f;

    sixtron::target_speeds *base_motor_speeds = &speeds_nulls;

    while (true) {

        // Update RBDC (will automatically update odometry, motor base, QEI, motors, PIDs...)
        // Wait for asserv tick
        controlThreadFlag.wait_any(CONTROL_THREAD_FLAG);

        /// CHECKING MODE
        //        if (current_mode == robot_mode::stop_now) {
        //            rbdc_poki->stop();
        //            led_out_red = 1;
        //            led_out_green = 0;
        //        } else if (current_mode == robot_mode::return_to_base) {
        //            rbdc_poki->start();
        //            ignore_lidar = false;
        //            checkLidar();
        //            rbdc_poki->setTarget(+0.2f, 0.0f, 0.0f);
        //        } else if (current_mode == robot_mode::match_run) {
        //            rbdc_poki->start();
        //            checkLidar();
        //        } else if (current_mode == robot_mode::recover_from_block) {
        //        }

        rbdc_poki->start(); // restart lidar if it was paused
        checkLidar(); // pause if lidar triggered

        // Update RBDC
        rbdc_result = rbdc_poki->update();

        // odom->update();
        //
        // if (user_switch_color) {
        //     base_motor_speeds = &debug_base;
        // } else {
        //     base_motor_speeds = &speeds_nulls;
        // }
        // basePokibot->setTargetSpeeds(*base_motor_speeds);
        // basePokibot->update();
        //
        // // Update time passed in control loop
        // time_passed += dt_pid;

        //        terminal_printf("%d\n", rbdc_result);
        //        terminal_debug("t=%6.2fs, %s\n", time_passed, rbdc_status[rbdc_result].c_str());

#if PRINTF_DEBUG_ENABLE
        static int print_rbdc_result = 0;
        static uint32_t timestamp = 1627551892437;
        if (print_rbdc_result >= 50) {
            print_rbdc_result = 0;
            // terminal_printf("%s\n", rbdc_status[rbdc_result].c_str());
            // terminal_printf("x=%dmm y=%dmm o=%drad %s\n",
            //         int(odom->getX() * 1000.0f),
            //         int(odom->getY() * 1000.0f),
            //         int(odom->getTheta() * 10000.f),
            //         rbdc_status[rbdc_result].c_str());

            // Use https://teleplot.fr/ for trajectory debug
            // terminal_printf(">Trajectory:%f:%f§m|xy\n>Status:%s|t\n",
            //         odom->getX(),
            //         odom->getY(),
            //         rbdc_status[rbdc_result].c_str());

            // terminal_printf(">Trajectory:%f:%f§m|xy\n>Status:%d\n",
            //         odom->getX(),
            //         odom->getY(),
            //         rbdc_result);

            terminal_printf(">Trajectory:%f:%f§m|xy\n>Angle_current:%d:%f§rad\n>Angle_target:%d:%"
                            "f§rad\n>Status:%s|t\n>RBDC_Result:%d\n",
                    odom->getX(),
                    odom->getY(),
                    timestamp,
                    fmodf(odom->getTheta(), 2 * M_PI_F),
                    timestamp,
                    rbdc_poki->getTarget().pos.theta,
                    rbdc_status[rbdc_result].c_str(),
                    rbdc_result);
        }
        print_rbdc_result++;
        timestamp++;

#endif
    }
}

void start_robot_pokibot_control_thread() {

    // Setup and start RBDC thread
    controlThread.start(control);
    ThisThread::sleep_for(500ms);

    // Setup and start Lidar thread
    lidarThread.start(lidarMain);
    ThisThread::sleep_for(500ms);
}
