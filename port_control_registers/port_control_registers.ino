// ------------------ DEFINITIONS ------------------------

// Motor and H-bdirge

#define PIN_ENA   3
#define PIN_ENB   6

#define BIT_IN1   4   // D2 - P104
#define BIT_IN2   6   // D4 - P106
#define BIT_IN3   7   // D5 - P107
#define BIT_IN4   12  // D7 - P112

#define dead_zone 90


// Ultrasonic 

#define PIN_ECHO  10
#define PIN_TRIG  11

#define BIT_TRIG  2   // D11 - P102
#define BIT_ECHO  3   // D10 - P103

#define OBSTACLE_DISTANCE 5

// LDR 

#define PIN_LDR   A0
#define BLACK     90
#define GRAY      120
#define WHITE     150

void  setup() 
{

  Serial.begin(9600);

  R_PORT1->PDR |= (1 << BIT_IN1) | (1 << BIT_IN2) |
                  (1 << BIT_IN3) | (1 << BIT_IN4) |
                  (1 << BIT_TRIG);

  R_PORT1->PODR &= ~(1 << BIT_ECHO);

  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                    (1 << BIT_IN3) | (1 << BIT_IN4))|
                    (1 << BIT_TRIG);
  
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
}

void  loop() 
{
  if (is_obstacle())
  {
    stop(); 
  }
  else
  {
    drive(150, 0);
  }
}

void  set_right_motor(int speed) 
{
  if (speed > 0) 
  {
    R_PORT1->PODR &= ~(1 << BIT_IN1);
    R_PORT1->PODR |=  (1 << BIT_IN2);
  } else 
  {
    R_PORT1->PODR |=  (1 << BIT_IN1);
    R_PORT1->PODR &= ~(1 << BIT_IN2);
    speed = -speed;
  }
  analogWrite(PIN_ENA, constrain(speed, dead_zone, 255));
}

void  set_left_motor(int speed) 
{
  if (speed > 0) 
  {
    R_PORT1->PODR &= ~(1 << BIT_IN3);
    R_PORT1->PODR |=  (1 << BIT_IN4);
  } else 
  {
    R_PORT1->PODR |=  (1 << BIT_IN3);
    R_PORT1->PODR &= ~(1 << BIT_IN4);
    speed = -speed;
  }
  analogWrite(PIN_ENB, constrain(speed, dead_zone, 255));
}

void  drive(int right_speed, int left_speed) 
{
  set_right_motor(right_speed);
  set_left_motor(left_speed);
}

void  stop()
{
  R_PORT1->PODR &= ~(1 << BIT_IN1);
  R_PORT1->PODR &= ~(1 << BIT_IN2);

  R_PORT1->PODR &= ~(1 << BIT_IN3);
  R_PORT1->PODR &= ~(1 << BIT_IN4);
}

int   get_distance_cm()
{
  R_PORT1->PODR &= ~(1 << BIT_TRIG);
  delayMicroseconds(2);
  R_PORT1->PODR |= (1 << BIT_TRIG);
  delayMicroseconds(10);
  R_PORT1->PODR &= ~(1 << BIT_TRIG);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH);
  int distance = (duration * 0.0343) / 2;
  Serial.println(distance);
  return  distance;
}

bool  is_obstacle()
{
  return get_distance_cm() < OBSTACLE_DISTANCE;
}
