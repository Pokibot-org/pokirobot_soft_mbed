/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "RBDC.h"
#include "mbed.h"
#include "motor_base_pokibot.h"
#include "motor_sensor_AS5047p.h"
#include "odometry_pokibot.h"

// Blinking rate in milliseconds
#define BLINKING_RATE 100ms
DigitalOut led_out_green(PC_8);
DigitalOut led_out_red(PB_4);
DigitalIn user_button(BUTTON1);

// Set up printf over STLINK
static UnbufferedSerial terminal(CONSOLE_TX, CONSOLE_RX, 115200);
Thread terminalThread(osPriorityBelowNormal, OS_STACK_SIZE * 4);
EventQueue terminalEventQueue;
static char terminal_shell_buff[200];

FileHandle *mbed::mbed_override_console(int fd) {
    return &terminal;
}

void terminal_printf(const char *fmt, ...);

/* #################################################################################################
 */

// Main Debug, just for the example. See main loop.
#define MAIN_THREAD_RATE 10ms
#define MAIN_THREAD_FLAG 0x01
Ticker mainThreadTicker;
EventFlags mainThreadFlag;

// BASE CONTROL UPDATE
#define CONTROL_THREAD_RATE 1ms
#define CONTROL_THREAD_FLAG 0x02
Ticker controlThreadTicker;
EventFlags controlThreadFlag;
Thread controlThread(osPriorityRealtime, OS_STACK_SIZE * 4);
static float dt_pid = 0.0f, hz_pid = 0.0f;
float robot_target_X = 0.0f, robot_target_Y = 0.0f, robot_target_theta = 0.0f;

// RBDC
sixtron::RBDC *rbdc_poki;
sixtron::RBDC_params rbdc_poki_params;
string rbdc_status[6] = {
    "RBDC_standby",
    "RBDC_done",
    "RBDC_correct_f_angle",
    "RBDC_moving",
    "RBDC_moving_&_angle",
    "RBDC_correct_i_angle",
};

// ENCODERS
#define ENC_RESOLUTION 16384
#define MOTOR_REDUCTION 50
#define ENC_WHEEL_RADIUS (0.07f / 2.0f)
#define ENC_WHEELS_DISTANCE (0.315f)
SPI spiAS5047p(PA_7, PA_6, PA_5); // mosi, miso, sclk

// ODOMETRY
sixtron::MotorSensorAS5047P *sensorLeft;
sixtron::MotorSensorAS5047P *sensorRight;
sixtron::OdometryPokibot *odom;

// MOTORS
sixtron::MotorBasePokibot *basePokibot;

// PID_DV & PID_THETA PRECISION
// #define PID_TETA_PRECISION  0.0872665f // 5°
#define PID_TETA_PRECISION 0.017453f // 1°
#define PID_DV_PRECISION 0.005f // 0.5 cm

// LORA
static UnbufferedSerial lora(PC_6, PC_7, 9600);
Thread loraThread(osPriorityBelowNormal, OS_STACK_SIZE);
EventQueue loraEventQueue;

/* #################################################################################################
 */

void mainThreadUpdate() {
    mainThreadFlag.set(MAIN_THREAD_FLAG);
}

/* #################################################################################################
 */

void controlThreadUpdate() {
    controlThreadFlag.set(CONTROL_THREAD_FLAG);
}

void control() {

    float time_passed = 0.0f;
    // Convert current rate of the loop in seconds (float)
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(CONTROL_THREAD_RATE);
    dt_pid = f_secs.count();
    hz_pid = 1.0f / dt_pid; // Very important for all PIDs

    // create encoders
    sensorLeft = new sixtron::MotorSensorAS5047P(&spiAS5047p,
            PA_8,
            dt_pid,
            ENC_RESOLUTION,
            ENC_RESOLUTION * MOTOR_REDUCTION,
            ENC_WHEEL_RADIUS,
            DIR_INVERTED);

    sensorRight = new sixtron::MotorSensorAS5047P(&spiAS5047p,
            PB_2,
            dt_pid,
            ENC_RESOLUTION,
            ENC_RESOLUTION * MOTOR_REDUCTION,
            ENC_WHEEL_RADIUS,
            DIR_NORMAL);

    // Create odometry. Will be init by RBDC.
    odom = new sixtron::OdometryPokibot(hz_pid, sensorLeft, sensorRight);

    // Create robot base. This will init all motors as well.  Will be init by RBDC.
    basePokibot = new sixtron::MotorBasePokibot(dt_pid, sensorLeft, sensorRight);

    rbdc_poki_params.rbdc_format = sixtron::RBDC_format::two_wheels_robot;
    rbdc_poki_params.max_output_dv = MAX_MOTOR_PWM - 0.2f;
    rbdc_poki_params.max_output_dtheta = MAX_MOTOR_PWM + 0.2f;
    rbdc_poki_params.can_go_backward = true;
    rbdc_poki_params.dt_seconds = dt_pid;
    rbdc_poki_params.final_theta_precision = PID_TETA_PRECISION;
    rbdc_poki_params.moving_theta_precision = 3 * PID_TETA_PRECISION;
    rbdc_poki_params.target_precision = 2 * PID_DV_PRECISION;
    rbdc_poki_params.dv_precision = PID_DV_PRECISION;

    rbdc_poki_params.pid_param_dteta.Kp = 2.5f;
    rbdc_poki_params.pid_param_dteta.Ki = 0.001f;
    rbdc_poki_params.pid_param_dteta.Kd = 0.0f;

    rbdc_poki_params.pid_param_dv.Kp = 1.6f;
    rbdc_poki_params.pid_param_dv.Ki = 0.001f;
    rbdc_poki_params.pid_param_dv.Kd = 0.0f;
    //    rbdc_poki_params.pid_param_dv.ramp = 0.5f * dt_pid;

    rbdc_poki = new sixtron::RBDC(odom,
            basePokibot,
            rbdc_poki_params); // will init odom and robot base as well
    sixtron::position target_pos;

    int square_state = 0;

    while (true) {
        // Wait for asserv tick
        controlThreadFlag.wait_any(CONTROL_THREAD_FLAG);

        // Update standby mode
        if (user_button.read() == 1) {
            rbdc_poki->stop();
        } else {
            rbdc_poki->start();
        }

        // Update Target
        target_pos.x = robot_target_X;
        target_pos.y = robot_target_Y;
        target_pos.theta = robot_target_theta;
        rbdc_poki->setTarget(target_pos);

        // Update RBDC
        int rbdc_result = rbdc_poki->update();

        // Do a square indefinitely
        if (rbdc_result == sixtron::RBDC_status::RBDC_done) {
            square_state++;

            switch (square_state) {
                case 1:
                    robot_target_X = 0.0f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = 0.0f;
                    break;
                case 2:
                    robot_target_X = 0.5f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = -1.57f;
                    break;
                case 3:
                    robot_target_X = 0.5f;
                    robot_target_Y = -0.5f;
                    robot_target_theta = -3.14f;
                    break;
                case 4:
                    robot_target_X = 0.0f;
                    robot_target_Y = -0.5f;
                    robot_target_theta = +1.57f;
                    break;
                default:
                    robot_target_X = 0.0f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = 0.0f;
                    square_state = 1;
                    break;
            }
        }

        // Update time passed in control loop
        time_passed += dt_pid;
    }
}

/* #################################################################################################
 */

void terminal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsprintf(terminal_shell_buff, fmt, args);
    va_end(args);

    terminal.write(terminal_shell_buff, strlen(terminal_shell_buff));
}

void process_terminalRX(char carac) {
    lora.write(&carac,1);
}

void rxTerminalCallback() {

    char c;
    if (terminal.read(&c, 1)) {
        terminalEventQueue.call(process_terminalRX, c);
    }
}

/* ############################### LORAAAAA ########################################################
 */

void process_LoraRX(char carac) {
    terminal_printf("%c",carac);
}

void rxLoraCallback() {

    char c;
    if (lora.read(&c, 1)) {
        loraEventQueue.call(process_LoraRX, c);
    }
}

/* #################################################################################################
 */

int main() {

    // Begin init
    led_out_green = 0;
    led_out_red = 1;
    terminal_printf("\nInit...\n");
    ThisThread::sleep_for(1000ms);

    float time_passed = 0.0f;
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(MAIN_THREAD_RATE);
    float time_incr = f_secs.count();

    // Setup main
    mainThreadTicker.attach(&mainThreadUpdate, MAIN_THREAD_RATE);

    // Setup asserv update
    controlThread.start(control);
    controlThreadTicker.attach(&controlThreadUpdate, CONTROL_THREAD_RATE);

    // Setup Serial Thread
    terminalThread.start(callback(&terminalEventQueue, &EventQueue::dispatch_forever));
    terminal.attach(&rxTerminalCallback);

    // Setup Lora Thread
    loraThread.start(callback(&loraEventQueue, &EventQueue::dispatch_forever));
    lora.attach(&rxLoraCallback);

    // Done init
    led_out_red = 0;
    led_out_green = 1;
    terminal_printf("Init Done.\n");

    while (true) {
        mainThreadFlag.wait_any(MAIN_THREAD_FLAG);

        led_out_red = !!user_button.read();

        time_passed += time_incr;
    }
}
