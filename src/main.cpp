/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "common.h"
#include "robot_pokibot.h"
#include "servo.h"

// Set up printf over STLINK
// Thread terminalThread(osPriorityBelowNormal, OS_STACK_SIZE);
// EventQueue terminalEventQueue;

// Afficheur 7 seg
static UnbufferedSerial afficheur(SEG7_RX, SEG7_TX, 9600);

/* #################################################################################################
 */

// Main Debug, just for the example. See main loop.
#define MAIN_THREAD_RATE 10ms
#define MAIN_THREAD_FLAG 0x01
Ticker mainThreadTicker;
EventFlags mainThreadFlag;

static Timeout ending;
static Timeout returning_to_base;

/* #################################################################################################
 */

void mainThreadUpdate() {
    mainThreadFlag.set(MAIN_THREAD_FLAG);
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
    start_robot_pokibot_control_thread();
    ThisThread::sleep_for(1000ms);

    // Setup Serial Thread
    //    terminalThread.start(callback(&terminalEventQueue, &EventQueue::dispatch_forever));
    //    terminal.attach(&rxTerminalCallback);

    // Done init
    led_out_red = 0;
    led_out_green = 1;

    // Servo
    servosTimerInit();
    servoSetPwmDuty(SERVO0, 1500);

    // end
    terminal_printf("Init Done.\n");

    // wait for tirette
    while (tirette)
        ;
    led_out_red = 1;
    led_out_green = 1;

    // Set current robot mode
    current_mode = robot_mode::match_run;

    //    returning_to_base.attach(&return_base_process, 80s);
    //    ending.attach(&end_process, 98s);

    robot_set_score(0);
    //
    //    // On est au fond de la zone, on avance pour aller gerber les balles
    //    ignore_lidar = true;
    //    robot_goto(0.35f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //
    //    // On lache les balles
    //    ThisThread::sleep_for(1s);
    //    servoSetPwmDuty(SERVO0, 3500);
    //    ThisThread::sleep_for(3s);
    //    robot_set_score(20);
    //
    //    // on recule un peu et on fonce dans le mur pour faire tomber les balles x2
    //    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //
    //    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //
    //    // on se remet un peu mieux sur la zone de départ avant de reset
    //    robot_goto(-0.05f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //
    //    // on reset l'odom
    //    rbdc_poki->setAbsolutePosition(0.0f, 0.0f, 0.0);
    //    ThisThread::sleep_for(200ms);
    //    rbdc_poki->setAbsolutePosition(0.0f, 0.0f, 0.0);
    //    ThisThread::sleep_for(200ms);
    //
    //    // On recule et on sort de la zone, on pousse des palets jusqu'à la prochaine assiette,
    //    petite
    //    // correction d'angle au passage
    //    robot_goto(-0.34f, 0.00f, 0.80f);
    //    robot_goto(-0.34f, 0.00f, 0.00f);
    //    ignore_lidar = false;
    //
    //    robot_goto(-1.6f, 0.05f, 0.0f);
    //    robot_set_score(35);
    //
    //    // On sort de a deuxième assiète, pour pouvoir compter les points palets
    //    robot_goto(0.25f, 0.0f, 0.0f, sixtron::RBDC_reference::relative);
    //    robot_set_score(26);
    //
    //    // On revient à la zone de départ
    //    ignore_lidar = false;
    //    robot_goto(-0.3f, 0.0f, 0.0f);
    //    ignore_lidar = true;
    //    robot_goto(+0.2f, 0.0f, 0.0f);
    //    robot_set_score(41);
    //    ignore_lidar = false;
    //    ThisThread::sleep_for(2s);
    //    robot_set_score(41);

    //    set_ignore_lidar(true);

    //    robot_goto(0.0f, 0.0f,  1.57f);
    //    ThisThread::sleep_for(2s);
    //    robot_goto(0.0f, 0.0f,  3.14f);
    //    ThisThread::sleep_for(2s);
    //    robot_goto(0.0f, 0.0f, -1.57f);
    //    ThisThread::sleep_for(2s);
    //    robot_goto(0.0f, 0.0f, 0.0f);
    //    ThisThread::sleep_for(2s);

    // square size
    static float square_size = 1.0f;
    static rtos::Kernel::Clock::duration_u32 time_wait = 1s;

    // robot speed
    robot_normal_speed();

    while (true) {
        mainThreadFlag.wait_any(MAIN_THREAD_FLAG);
        //        led_out_green = 1;
        //        robot_goto(0.5f, 0.0f, -1.57f);
        //        led_out_green = 0;
        //        ThisThread::sleep_for(2s);
        //        led_out_green = 1;
        //        robot_goto(0.5f, 0.5f, -3.14f);
        //        led_out_green = 0;
        //        ThisThread::sleep_for(2s);
        //        led_out_green = 1;
        //        robot_goto(0.0f, 0.5f, +1.57f);
        //        led_out_green = 0;
        //        ThisThread::sleep_for(2s);
        //        led_out_green = 1;
        //        robot_goto(0.0f, 0.0f, 0.0f);
        //        led_out_green = 0;
        //        ThisThread::sleep_for(2s);

        //        robot_goto(0.0f, 0.0f, -1.57f);
        //        ThisThread::sleep_for(2s);
        //        robot_goto(0.0f, 0.0f, -3.14f);
        //        ThisThread::sleep_for(2s);
        //        robot_goto(0.0f, 0.0f, +1.57f);
        //        ThisThread::sleep_for(2s);
        //        robot_goto(0.0f, 0.0f, 0.0f);

        // FULL SQUARE
        robot_goto(square_size, 0.0);
        ThisThread::sleep_for(time_wait);
        robot_goto(square_size, square_size);
        ThisThread::sleep_for(time_wait);
        robot_goto(0.0, square_size);
        ThisThread::sleep_for(time_wait);
        robot_goto(0.0, 0.0);
        ThisThread::sleep_for(time_wait);

        // ANGULAR ONLY
        // robot_goto(0.0, 0.0, DEG_TO_RAD(-90.0f));
        // ThisThread::sleep_for(time_wait);
        // robot_goto(0.0, 0.0, DEG_TO_RAD(-180.0f));
        // ThisThread::sleep_for(time_wait);
        // robot_goto(0.0, 0.0, DEG_TO_RAD(-270.0f));
        // ThisThread::sleep_for(time_wait);
        // robot_goto(0.0, 0.0, DEG_TO_RAD(0.0f));
        // ThisThread::sleep_for(time_wait);

        // LINEAR ONLY
        // robot_goto(2.0f, 0.0f, 0.0f);
        // ThisThread::sleep_for(time_wait);
        // robot_goto(0.0f, 0.0f, 0.0f);
        // ThisThread::sleep_for(time_wait);

        // nothing to do after the strat
    }
}
