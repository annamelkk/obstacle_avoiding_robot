#include <Servo.h>

#define TRIG A0
#define ECHO A1
#define OBSTACLE_DISTANCE 5
#define SONAR_INTERVAL 60 // Increased slightly for echo stability

#define BIG 3
#define SMALL 5

#define HORIZONTAL  180
#define VERTICAL    100
#define OPEN        0
#define CLOSED      90
#define SERVO_DELAY 10

Servo big;
Servo small;

unsigned long last_sonar = 0;
int DISTANCE = 999;

void setup() {
  Serial.begin(9600);
  big.attach(BIG, 544, 2500);
  small.attach(SMALL);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
 neutral_position();
  delay(1000);
}

void loop() {
 grab_bottle();
}

void neutral_position()
{
  small.write(CLOSED);
  big.write(HORIZONTAL);
}


void grab_bottle()
{
  move_arm(-1);
  delay(1000);
  move_hand(-1);
  delay(2000);
  move_hand(1);
  delay(1000);
  move_arm(1);
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
