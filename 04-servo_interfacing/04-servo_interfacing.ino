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
#define TRIG A3
#define ECHO A4
#define OBSTACLE_DISTANCE 15
#define SONAR_INTERVAL 40
#define DRIVE_SPEED 150

int DISTANCE = 999;

// servos
#define BIG 10
#define SMALL 11

#define HORIZONTAL  180
#define VERTICAL    100
#define OPEN        0
#define CLOSED      90
#define SERVO_DELAY 10

#define REMOVAL_DELAY 600

// buzzer
#define BUZZER 8

Servo big;
Servo small;

unsigned long last_sonar = 0;

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  big.attach(BIG, 544, 2500);
  small.attach(SMALL);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(BUZZER, OUTPUT);
  
  stop();
  neutral_position();
  delay(1000);
}

void loop() 
{
  unsigned long now = millis();

  if (now - last_sonar >= SONAR_INTERVAL)
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
    return;
  }
  Serial.print("Distance:"); Serial.println(DISTANCE);
  drive(DRIVE_SPEED, DRIVE_SPEED);
}


// =================== OBSTACLE REMOVING ===================

void neutral_position()
{
  small.write(CLOSED);
  big.write(VERTICAL);
}


void move_hand(int dir)
{ // dir = 1 close the hand dir = -1 open the hand
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
  // 1 == vertical -1 == horizontal
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

  // removal side is hardcoded as there are no sensors to detect
  drive(DRIVE_SPEED, DRIVE_SPEED+20);
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

  drive(-DRIVE_SPEED, -(DRIVE_SPEED+20));
  delay(REMOVAL_DELAY);
}

// =================== SONAR ================================
int get_distance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // pulseIn returns duration in microseconds
  unsigned long duration = pulseIn(ECHO, HIGH, 25000); 
  
  if (duration == 0) {
    return 999; // Return out-of-range value if no echo
  }
  
  // Speed of sound calculation
  int d = (duration * 0.0343) / 2;
  return d;
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
