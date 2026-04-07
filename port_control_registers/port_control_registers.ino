// ------------------ DEFINITIONS ------------------------

// Motors

#define PIN_ENA 3
#define PIN_ENB 6

#define BIT_ENA 4 // D3
#define BIT_ENB 6 // D6
#define BIT_IN1 5 // D2
#define BIT_IN2 3 // D4
#define BIT_IN3 2 // D5
#define BIT_IN4 7 // D7

#define dead_zone 90

// Sound sensor

void setup() 
{

  // motor control pins are outputs (PDR = Port Direction Register)
  R_PORT1->PDR |= (1 << BIT_ENA) | (1 << BIT_ENB) |
                  (1 << BIT_IN1) | (1 << BIT_IN2) |
                  (1 << BIT_IN3) | (1 << BIT_IN4);

  // turn off motors (PODR = Port Output Data Register)
  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) | 
                     (1 << BIT_IN3) | (1 << BIT_IN4));
}

void loop() 
{
  drive(150, 150); // Test forward
}

// ---------------- FUNCTIONS --------------------

void set_right_motor(int speed) 
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

void set_left_motor(int speed) 
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

void drive(int right_speed, int left_speed) 
{
  set_right_motor(right_speed);
  set_left_motor(left_speed);
}

