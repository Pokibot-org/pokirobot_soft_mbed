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
    pid_motor_params.Kp = 3.0f;
    //        pid_motor_params.Ki = 40.0f; // 5.0
    pid_motor_params.Ki = 0.5f; // 5.0
    pid_motor_params.Kd = 0.00f;
    pid_motor_params.dt_seconds = dt_pid;
    pid_motor_params.ramp_high = 2.0f; // acc_max when ramping up (positive or negative) in [m/s²].
    pid_motor_params.ramp_low = 4.0f; // acc_max when ramping down (positive or negative) in [m/s²].

    // Create robot base. This will init all motors as well.  Will be init by RBDC.
    basePokibot = new sixtron::MotorBasePokibot(
            dt_pid, sensorLeft, sensorRight, pid_motor_params, ENC_WHEELS_DISTANCE, MAX_MOTOR_PWM);

    // Setup RBDC
    sixtron::RBDC_params rbdc_poki_params;
    rbdc_poki_params.rbdc_format = sixtron::RBDC_format::two_wheels_robot;
    rbdc_poki_params.max_output_dv = 1.0f;
    rbdc_poki_params.max_output_dtheta = 8.0f;
    rbdc_poki_params.can_go_backward = true;
    rbdc_poki_params.dt_seconds = dt_pid;
    rbdc_poki_params.final_theta_precision = 3 * ONE_DEGREE_IN_RAD;
    rbdc_poki_params.moving_theta_precision = 10 * ONE_DEGREE_IN_RAD;
    rbdc_poki_params.target_precision = 4 * PID_DV_PRECISION;
    rbdc_poki_params.dv_precision = 2 * PID_DV_PRECISION;

    rbdc_poki_params.pid_param_dteta.Kp = 2.5f;
    rbdc_poki_params.pid_param_dteta.Ki = 8.0f;
    rbdc_poki_params.pid_param_dteta.Kd = 0.0f;

    rbdc_poki_params.pid_param_dv.Kp = 1.0f;
    rbdc_poki_params.pid_param_dv.Ki = 0.001f;
    rbdc_poki_params.pid_param_dv.Kd = 0.0f;
    //    rbdc_poki_params.pid_param_dv.ramp = 0.2f * dt_pid;

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

        // Update RBDC
        rbdc_result = rbdc_poki->update();

        // Update time passed in control loop
        time_passed += dt_pid;

        //        terminal_printf("%d\n", rbdc_result);
        //        terminal_debug("t=%6.2fs, %s\n", time_passed, rbdc_status[rbdc_result].c_str());

#ifdef PRINTF_DEBUG_ENABLE
        static int loop_debug = 100;
        loop_debug--;
        if (loop_debug <= 0) {
            loop_debug = 100;
            terminal_debug("X=%2.3fm, Y=%2.3fm, O=%2.3frad, %s\n",
                    odom->getX(),
                    odom->getY(),
                    odom->getTheta(),
                    rbdc_status[rbdc_result].c_str());
        }

#endif
    }
}

// MBED STARTING NEW THREAD
void start_robot_pokibot_control_thread() {
    controlThread.start(control);
    ThisThread::sleep_for(500ms);

    // Setup Lidar
    lidarThread.start(lidarMain);
    ThisThread::sleep_for(500ms);
}