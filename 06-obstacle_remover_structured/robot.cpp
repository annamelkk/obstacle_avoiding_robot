#include "robot.h"

// ================= GLOBAL OBJECTS =================

Servo big;
Servo small;

// ================= CONTROLLER MODE ================

ControlMode current_mode = PID_MODE;

// ================= GLOBAL VARIABLES ===============

float error = 0;
float prev_error = 0;
float integral = 0;
float derivative = 0;

unsigned long last_pid_time = 0;

// ================= SETUP ==========================

void setup_pins()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    pinMode(LDR, INPUT);

    pinMode(BUZZER, OUTPUT);

    big.attach(BIG, 544, 2500);
    small.attach(SMALL);
}

// ================= CONTROLLER SELECTOR ============

void run_controller(int ldr)
{
    switch(current_mode)
    {
        case ON_OFF_MODE:
            ON_OFF_CTRL(ldr);
            break;

        case P_MODE:
            P_CTRL(ldr);
            break;

        case PID_MODE:
            PID_CTRL(ldr);
            break;
    }
}

// ================= ON/OFF CONTROLLER ==============


void ON_OFF_CTRL(int ldr)
{
    if (ldr < GRAY - THRESHOLD)
    {
        analogWrite(ENA, DZ + ON_CORRECT);
        analogWrite(ENB, 0);
    }
    else if (ldr > GRAY + THRESHOLD)
    {
        analogWrite(ENB, DZ + ON_CORRECT);
        analogWrite(ENA, 0);
    }
    else
    {
        analogWrite(ENA, DZ + ON_CORRECT);
        analogWrite(ENB, DZ + ON_CORRECT);
    }
}

// ================= P CONTROLLER ===================

void P_CTRL(int ldr)
{
    error = ldr - GRAY;

    int left  = P_DRIVE_SPEED + (P_KP * error);
    int right = P_DRIVE_SPEED - (P_KP * error);

    drive(left, right);
}


// ================= PID CONTROLLER =================

void PID_CTRL(int ldr)
{
    unsigned long now = millis();
    float dt = (now - last_pid_time) / 1000.0;

    if (dt <= 0)
        return;

    last_pid_time = now;

    error = ldr - GRAY;

    integral += error * dt;
    integral = constrain(integral, -20, 20);

    derivative = (error - prev_error) / dt;

    float adjust =
        (K_P * error) +
        (K_I * integral) +
        (K_D * derivative);

    prev_error = error;

    adjust = constrain(adjust, -50, 50);

    drive(DRIVE_SPEED + adjust,
          DRIVE_SPEED - adjust);
}

// ================= MOTOR FUNCTIONS ================

void set_right_motor(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        analogWrite(ENA,
                     map(speed, 0, 255, DZ, 255));
    }
    else if (speed < 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        analogWrite(ENA,
                     map(-speed, 0, 255, DZ, 255));
    }
    else
    {
        analogWrite(ENA, 0);
    }
}

void set_left_motor(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        analogWrite(ENB,
                     map(speed, 0, 255, DZ, 255));
    }
    else if (speed < 0)
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        analogWrite(ENB,
                     map(-speed, 0, 255, DZ, 255));
    }
    else
    {
        analogWrite(ENB, 0);
    }
}

void drive(int l, int r)
{
    set_left_motor(l);
    set_right_motor(r);
}

void stop()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// ================= SONAR ==========================

int get_distance()
{
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long duration = pulseIn(ECHO,
                                     HIGH,
                                     30000);

    if (duration == 0)
        return 999;

    return (duration * 0.0343) / 2;
}

// ================= ARM CONTROL ====================

void neutral_position()
{
    small.write(CLOSED);
    big.write(VERTICAL);
}

void move_hand(int dir)
{
    if (dir == 1)
    {
        for (int i = OPEN; i <= CLOSED; i++)
        {
            small.write(i);
            delay(SERVO_DELAY);
        }
    }
    else if (dir == -1)
    {
        for (int i = CLOSED; i >= OPEN; i--)
        {
            small.write(i);
            delay(SERVO_DELAY);
        }
    }
}

void move_arm(int dir)
{
    if (dir == 1)
    {
        for (int i = HORIZONTAL;
                 i >= VERTICAL;
                 i--)
        {
            big.write(i);
            delay(SERVO_DELAY);
        }
    }
    else if (dir == -1)
    {
        for (int i = VERTICAL;
                 i <= HORIZONTAL;
                 i++)
        {
            big.write(i);
            delay(SERVO_DELAY);
        }
    }
}

// ================= OBSTACLE REMOVAL ===============

void grab_bottle()
{
    drive(DRIVE_SPEED,
          DRIVE_SPEED);

    delay(500);

    stop();

    move_hand(-1);
    delay(1000);

    move_arm(-1);
    delay(1000);

    move_hand(1);
    delay(2000);

    move_arm(1);
}

void remove_bottle()
{
    stop();
    delay(1000);

    drive(REMOVAL_SPEED,
          REMOVAL_SPEED + 50);

    delay(REMOVAL_DELAY);

    stop();
    delay(500);
}
