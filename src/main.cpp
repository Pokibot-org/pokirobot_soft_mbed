/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "RBDC.h"
#include "common.h"
#include "lidar_serial.h"
#include "motor_base_pokibot.h"
#include "motor_sensor_AS5047p.h"
#include "odometry_pokibot.h"
#include "servo.h"

// Blinking rate in milliseconds
#define BLINKING_RATE 100ms
DigitalOut led_out_green(LED_GREEN);
DigitalOut led_out_red(LED_RED);
DigitalIn user_button(BUTTON1);
DigitalIn tirette(TIRETTE);

// Set up printf over STLINK
static UnbufferedSerial terminal(CONSOLE_TX, CONSOLE_RX, 921600);
Thread terminalThread(osPriorityBelowNormal, OS_STACK_SIZE);
EventQueue terminalEventQueue;
static char terminal_shell_buff[200];

// Afficheur 7 seg
static UnbufferedSerial afficheur(SEG7_RX, SEG7_TX, 9600);

// Ticker ending;

FileHandle *mbed::mbed_override_console(int fd) {
    return &terminal;
}

void terminal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(terminal_shell_buff, fmt, args);
    va_end(args);

    terminal.write(terminal_shell_buff, length);
}

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
volatile int rbdc_result = sixtron::RBDC_status::RBDC_standby;
// volatile int STOP_NOW = 0;

// ENCODERS
#define ENC_RESOLUTION 16384
#define MOTOR_REDUCTION 50
#define ENC_WHEEL_RADIUS (0.07f / 2.0f)
#define ENC_WHEELS_DISTANCE (0.315f)
SPI spiAS5047p(ENC_MOSI, ENC_MISO, ENC_SCK); // mosi, miso, sclk

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

// LIDAR
// UnbufferedSerial serialLidar(PA_9, PA_10, 230400);
Thread lidarThread(osPriorityAboveNormal, OS_STACK_SIZE);

// ROBOT MODE
typedef enum {
    standby,
    match_run,
    recover_from_block,
    return_to_base,
    stop_now,
} robot_mode;

volatile robot_mode current_mode = standby;
volatile bool ignore_lidar = false;

static Timeout ending;
static Timeout returning_to_base;

/* #################################################################################################
 */

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

void control() {

    float time_passed = 0.0f;
    // Convert current rate of the loop in seconds (float)
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(CONTROL_THREAD_RATE);
    dt_pid = f_secs.count();
    hz_pid = 1.0f / dt_pid; // Very important for all PIDs

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
    rbdc_poki_params.pid_param_dv.ramp = 0.2f * dt_pid;

    rbdc_poki = new sixtron::RBDC(odom,
            basePokibot,
            rbdc_poki_params); // will init odom and robot base as well
    sixtron::position target_pos;
    rbdc_poki->setTarget(0.0f, 0.0f, 0.0f);

    while (true) {
        // Wait for asserv tick
        controlThreadFlag.wait_any(CONTROL_THREAD_FLAG);

        /// CHECKING MODE
        if (current_mode == robot_mode::stop_now) {
            rbdc_poki->stop();
            led_out_red = 1;
            led_out_green = 0;
        } else if (current_mode == robot_mode::return_to_base) {
            rbdc_poki->start();
            ignore_lidar = false;
            checkLidar();
            rbdc_poki->setTarget(+0.2f, 0.0f, 0.0f);
        } else if (current_mode == robot_mode::match_run) {
            rbdc_poki->start();
            checkLidar();
        } else if (current_mode == robot_mode::recover_from_block) {

        }

        // Update RBDC
        rbdc_result = rbdc_poki->update();

        // Update time passed in control loop
        time_passed += dt_pid;
    }
}

/* #################################################################################################
 */

void process_terminalTX(char carac) {
}

void rxTerminalCallback() {

    char c;
    if (terminal.read(&c, 1)) {
        terminalEventQueue.call(process_terminalTX, c);
    }
}

/* #################################################################################################
 */

void robot_goto(float x,
        float y,
        float theta,
        sixtron::RBDC_reference reference = sixtron::RBDC_reference::absolute) {

    rbdc_poki->setTarget(x, y, theta, reference);
    ThisThread::sleep_for(100ms);
    while (rbdc_result != sixtron::RBDC_status::RBDC_done)
        ;
}

char affich_buff[20];

void robot_set_score(int score) {

    int length_score = sprintf(affich_buff, "SET SCOR %d\r\n", score);
    afficheur.write(affich_buff, length_score);
    ThisThread::sleep_for(300ms);
    afficheur.write(affich_buff, length_score);
    ThisThread::sleep_for(300ms);
//    afficheur.write(affich_buff, length_score);
}

void return_base_process() {
    current_mode = robot_mode::return_to_base;
}

void end_process() {
    current_mode = robot_mode::stop_now;
}

int main() {

    // Begin init
    current_mode = robot_mode::standby;
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
    ThisThread::sleep_for(1000ms);

    // Setup Serial Thread
    terminalThread.start(callback(&terminalEventQueue, &EventQueue::dispatch_forever));
    terminal.attach(&rxTerminalCallback);

    // Setup Lidar
    lidarThread.start(lidarMain);

    // Done init
    led_out_red = 0;
    led_out_green = 1;
    terminal_printf("Init Done.\n");

    // Servo
    servosTimerInit();
    servoSetPwmDuty(SERVO0, 1500);

    while (tirette)
        ;

    // Set current robot mode
    current_mode = robot_mode::match_run;

    returning_to_base.attach(&return_base_process, 80s);
    ending.attach(&end_process, 98s);

    robot_set_score(0);

    // On est au fond de la zone, on avance pour aller gerber les balles
    ignore_lidar = true;
    robot_goto(0.35f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);

    // On lache les balles
    ThisThread::sleep_for(1s);
    servoSetPwmDuty(SERVO0, 3500);
    ThisThread::sleep_for(3s);
    robot_set_score(20);

    // on recule un peu et on fonce dans le mur pour faire tomber les balles x2
    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);

    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);

    // on se remet un peu mieux sur la zone de départ avant de reset
    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);

    // on reset l'odom
    rbdc_poki->setAbsolutePosition(0.0f, 0.0f, 0.0);
    ThisThread::sleep_for(200ms);
    rbdc_poki->setAbsolutePosition(0.0f, 0.0f, 0.0);
    ThisThread::sleep_for(200ms);

    // On recule et on sort de la zone, on pousse des palets jusqu'à la prochaine assiette, petite
    // correction d'angle au passage
    robot_goto(-0.34f, 0.00f, 0.80f);
    robot_goto(-0.34f, 0.00f, 0.00f);
    ignore_lidar = false;

    robot_goto(-1.6f, 0.05f, 0.0f);
    robot_set_score(35);

    // On sort de a deuxième assiète, pour pouvoir compter les points palets
    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    robot_set_score(26);

    // On revient à la zone de départ
    ignore_lidar = false;
    robot_goto(-0.3f, 0.0f, 0.0f);
    ignore_lidar = true;
    robot_goto(+0.2f, 0.0f, 0.0f);
    robot_set_score(41);
    ignore_lidar = false;
    ThisThread::sleep_for(2s);
    robot_set_score(41);

    while (true) {
        mainThreadFlag.wait_any(MAIN_THREAD_FLAG);

        // nothing to do after the strat
    }
}
