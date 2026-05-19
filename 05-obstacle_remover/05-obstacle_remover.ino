#include <Servo.h>

// ====================== DEFINITIONS ======================

#define DZ    90
#define ENA   3
#define ENB   6

// right motor
#define IN1   2 
#define IN2   4

// left motor
#define IN3   5
#define IN4   7

// ultrasonic
#define ECHO              A4
#define TRIG              A3
#define OBSTACLE_DISTANCE 15
#define SONAR_INTERVAL    50

// line sensor
#define LDR       A0
#define BLACK     650
#define WHITE     1023
#define GRAY      900
#define THRESHOLD 900

#define DRIVE_SPEED     50          // used for line following (PID)
#define REMOVAL_SPEED   130

// ON OFF
#define ON_CORRECT 30

// P CTRL
#define P_DRIVE_SPEED 90
#define P_KP 0.2

// PID
#define K_P 0.6
#define K_D 0.05
#define K_I 0.0


// ===== CONTROLLER SELECT =====
#define CTRL_ON_OFF 0
#define CTRL_P      1
#define CTRL_PID    2

#define CONTROL_MODE CTRL_PID

// ======= SERVOS =============
#define BIG   10
#define SMALL 11

#define HORIZONTAL  180
#define VERTICAL    100
#define OPEN        0
#define CLOSED      90
#define SERVO_DELAY 10

#define REMOVAL_DELAY 600

#define BUZZER 8

// ====================== GLOBALS ==========================

float error       = 0;
float prev_error  = 0;
float integral    = 0;
float derivative  = 0;

unsigned long last_pid_time = 0;
unsigned long last_sonar    = 0;
int DISTANCE                = 999;

Servo big;
Servo small;


// ====================== SETUP ============================

void setup() 
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LDR, INPUT);

  big.attach(BIG, 544, 2500);
  small.attach(SMALL);

  pinMode(BUZZER, OUTPUT);

  stop();
  neutral_position();
  delay(1000);

  last_pid_time = millis();
}


// ====================== LOOP =============================

void loop() 
{
  unsigned long now = millis();

  if (now - last_sonar > SONAR_INTERVAL)
  {
    last_sonar = now;
    DISTANCE = get_distance();
  }

  if (DISTANCE < OBSTACLE_DISTANCE)
  {
    stop();

    tone(BUZZER, 1000);
    delay(300);
    noTone(BUZZER);

    grab_bottle();
    remove_bottle();
    delay(100);

    last_pid_time = millis();
    integral = 0;
    return;
  }

  int ldr = analogRead(LDR);
  Serial.print("LDR: "); Serial.println(ldr);
  Serial.print("Distance: "); Serial.println(DISTANCE);

#if CONTROL_MODE == CTRL_ON_OFF
    ON_OFF_CTRL(ldr);
#elif CONTROL_MODE == CTRL_P
    P_CTRL(ldr);
#elif CONTROL_MODE == CTRL_PID
    PID_CTRL(ldr);
#endif
}


// ====================== ON OFF CTRL =====================

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


// ====================== P CTRL ==========================

void P_CTRL(int ldr)
{
  error = ldr - GRAY;

  int left  = P_DRIVE_SPEED + (P_KP * error);
  int right = P_DRIVE_SPEED - (P_KP * error);

  drive(left, right);
}


// ====================== PID CTRL (millis based) =========

void PID_CTRL(int ldr) 
{
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0;

  if (dt <= 0) return;

  last_pid_time = now;

  error = (ldr - GRAY);
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


// ====================== MOTOR ============================

void set_right_motor(int speed) 
{
  speed = constrain(speed, -255, 255);

  if (speed > 0) 
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, map(speed, 0, 255, DZ, 255));
  } 
  else if (speed < 0) 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, map(-speed, 0, 255, DZ, 255));
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
    analogWrite(ENB, map(speed, 0, 255, DZ, 255));
  } 
  else if (speed < 0) 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, map(-speed, 0, 255, DZ, 255));
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


// ====================== SONAR ===========================

int get_distance() 
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 30000);

  if (duration == 0) return 999;

  return (duration * 0.0343) / 2;
}


// =================== OBSTACLE REMOVING ===================

void neutral_position()
{
  small.write(CLOSED);
  big.write(VERTICAL);
}


void move_hand(int dir)
{ // dir = 1 close the hand, dir = -1 open the hand
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
  // 1 == vertical, -1 == horizontal
  if (dir == 1)
  {
    for (int i = HORIZONTAL; i >= VERTICAL; i--)
    {
      big.write(i);
      delay(SERVO_DELAY);
    }
  } 
  else if (dir == -1)
  {
    for (int i = VERTICAL; i <= HORIZONTAL; i++)
    {
      big.write(i);
      delay(SERVO_DELAY);
    }
  }
}


void grab_bottle()
{
  drive(DRIVE_SPEED, DRIVE_SPEED);
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

  // so motors don't stall under load during the turn
  drive(REMOVAL_SPEED - 50, REMOVAL_SPEED + 50);
  delay(REMOVAL_DELAY);
  stop();

  delay(500);
  move_arm(-1);
  delay(1000);
  move_hand(-1);
  delay(1000);
  move_arm(1);
  delay(1000);
  move_hand(1);
  delay(1000);

  drive(-(REMOVAL_SPEED - 50), -(REMOVAL_SPEED + 50));
  delay(REMOVAL_DELAY);
  stop();
  delay(200);
}
