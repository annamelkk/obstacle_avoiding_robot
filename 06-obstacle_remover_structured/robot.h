#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>

// ================= TUNING & CALIBRATION CONSTANTS =================
#define SONAR_INTERVAL    50
#define GRAY              500   // Midpoint value
#define THRESHOLD         50    // Deadband threshold for ON/OFF control
#define ON_CORRECT        40    // Extra speed boost for turning in ON/OFF mode

#define P_DRIVE_SPEED     50    // Base speed for P Controller
#define P_KP              0.5   // Proportional gain for P Controller

#define K_P               0.8   // PID Proportional Gain
#define K_I               0.05  // PID Integral Gain
#define K_D               0.1   // PID Derivative Gain

// ================= PIN DEFINITIONS =================

#define DZ    90
#define ENA   3
#define ENB   6

#define IN1   2
#define IN2   4
#define IN3   5
#define IN4   7

#define ECHO  A4
#define TRIG  A3

#define LDR   A0

#define BIG   10
#define SMALL 11

#define BUZZER 8

#define DRIVE_SPEED 50
#define REMOVAL_SPEED 130

#define OBSTACLE_DISTANCE 15

// =============== SERVO POSITIONS ===============

#define HORIZONTAL 180
#define VERTICAL   100
#define OPEN       0
#define CLOSED     90

#define SERVO_DELAY 10
#define REMOVAL_DELAY 600

// ================= CONTROLLER MODE ENUM =================

enum ControlMode {
    ON_OFF_MODE,
    P_MODE,
    PID_MODE
};

// ================= GLOBALS =====================

extern Servo big;
extern Servo small;
extern ControlMode current_mode;

extern unsigned long last_pid_time;
extern float integral;

// ================= FUNCTIONS ===================

// setup helpers
void setup_pins();
void neutral_position();

// movement
void set_left_motor(int speed);
void set_right_motor(int speed);
void drive(int l, int r);
void stop();

// sensors
int get_distance();

// arm
void move_hand(int dir);
void move_arm(int dir);

// obstacle removing
void grab_bottle();
void remove_bottle();

// controllers
void run_controller(int ldr);
void ON_OFF_CTRL(int ldr);
void P_CTRL(int ldr);
void PID_CTRL(int ldr);

#endif
