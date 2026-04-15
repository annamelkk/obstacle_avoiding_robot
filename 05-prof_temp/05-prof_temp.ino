// ====================== DEFINITIONS ======================

// Motor and H-bridge
#define PIN_ENA   3
#define PIN_ENB   6
#define BIT_IN1   4   // D2 - P104
#define BIT_IN2   6   // D4 - P106
#define BIT_IN3   7   // D5 - P107
#define BIT_IN4   12  // D7 - P112

// Ultrasonic
#define PIN_ECHO          10
#define PIN_TRIG          11
#define BIT_TRIG          11  // D11 - P411
#define BIT_ECHO          3   // D10 - P103
#define OBSTACLE_DISTANCE 30

// LDR
#define PIN_LDR       A0
#define BLACK         495
#define WHITE         780
#define THRESHOLD     ((BLACK + WHITE)/2)

#define SMOOT_FACTOR 5
#define DEADBAND     5

#define SEARCH_SPEED  120
#define DRIVE_SPEED   180
#define RECOVER_SPEED 100

// PID controller values
#define k_p           0.6
#define k_i           0.0
#define k_d           0.2

// ====================== GLOBALS =========================

// on off
int last_turn = 0;
int prev_state = 1; // assume starting black
int turn_strength = 100;

// pid
float error = 0;
float prev_error = 0;
float integral = 0;

// ====================== SETUP ===========================

/*
 * We are using poert registers to directly manipulate Arduino uno R4 Renesas MCUs memory,
 * according to Google this method is around 30x faster than digitalWrite() and pinMode()
 */
void setup()
{
  Serial.begin(9600);

  // motor direction pins – all on PORT1 in Renesas datasheet
  R_PORT1->PDR  |=  (1 << BIT_IN1) | (1 << BIT_IN2) |
                    (1 << BIT_IN3) | (1 << BIT_IN4);
  // start LOW
  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                     (1 << BIT_IN3) | (1 << BIT_IN4));

  // ultrasonic sensor
  R_PORT4->PDR  |=  (1 << BIT_TRIG); // TRIG output – PORT4 bit 11
  R_PORT1->PDR  &= ~(1 << BIT_ECHO); // ECHO input  – PORT1 bit 3
  R_PORT4->PODR &= ~(1 << BIT_TRIG); // TRIG starts LOW

  // PWM enable pins, using pinmode for analogWrite()
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  delay(1000); // 1 sec delay to place the robot in position
}

// ====================== LOOP ============================
void loop()
{
  int ldr = ldr_smoothing();
  
  //ON_OFF_CTRL(ldr);
  PID_CTRL(ldr);
  

}

// ====================== CONTROL FUNCTIONS ===============

void ON_OFF_CTRL(int ldr)
{
  int current_state = (ldr < THRESHOLD) ? 1 : 0;

  if (prev_state ==1 && current_state == 0) 
    last_turn = !last_turn; // flip direction after each direction loss

  prev_state = current_state;

  if (current_state == 1)
  {
    drive(DRIVE_SPEED, DRIVE_SPEED);
    turn_strength = 100;
  }
  else
  {
    turn_strength += 5;
    if (last_turn == 0)
      drive(SEARCH_SPEED, turn_strength);
    else 
      drive(turn_strength, SEARCH_SPEED);
  }
}

void PID_CTRL(int ldr) 
{
  error = THRESHOLD - ldr;
  static float derivative = error - prev_error;
  integral += error;

  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;

  float control = k_p * error + k_i * integral + k_d * derivative;

  if (error < -120 && abs(derivative) < 5)
  {
    // fully lost → force oscillation search
    control = (prev_error > 0) ? 100 : -100;
  }
  
  if (abs(control) < DEADBAND) control = 0;
  prev_error = error;

  int base_speed = 140;

  int left_speed  = base_speed + control;
  int right_speed = base_speed - control;

  // clamp speeds
  if (left_speed > 255) left_speed = 255;
  if (left_speed < 0) left_speed = 0;

  if (right_speed > 255) right_speed = 255;
  if (right_speed < 0) right_speed = 0;

  drive(left_speed, right_speed);
}


// ====================== MOTOR HELPERS ===================

void set_right_motor(int speed)
{
  // clamp speeds
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;

  // posivite - forward, negative backward
  if (speed > 0)
  {
    R_PORT1->PODR |=  (1 << BIT_IN1);
    R_PORT1->PODR &= ~(1 << BIT_IN2);
    analogWrite(PIN_ENA, speed);
  }
  else if (speed < 0)
  {
    R_PORT1->PODR &= ~(1 << BIT_IN1);
    R_PORT1->PODR |=  (1 << BIT_IN2);
    analogWrite(PIN_ENA, -speed);
  }
  else { analogWrite(PIN_ENA, 0); }
}

void set_left_motor(int speed)
{
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;
  if (speed > 0)
  {
    R_PORT1->PODR |=  (1 << BIT_IN3);
    R_PORT1->PODR &= ~(1 << BIT_IN4);
    analogWrite(PIN_ENB, speed);
  }
  else if (speed < 0)
  {
    R_PORT1->PODR &= ~(1 << BIT_IN3);
    R_PORT1->PODR |=  (1 << BIT_IN4);
    analogWrite(PIN_ENB, -speed);
  }
  else { analogWrite(PIN_ENB, 0); }
}

void drive(int r, int l)
{
  set_right_motor(r);
  set_left_motor(l);
}

void stop()
{
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                    (1 << BIT_IN3) | (1 << BIT_IN4));
}

// ====================== SENSOR HELPERS ==================

int get_distance_cm()
{
  // pull trig down then high then down
  R_PORT4->PODR &= ~(1 << BIT_TRIG);
  delayMicroseconds(2);
  R_PORT4->PODR |=  (1 << BIT_TRIG);
  delayMicroseconds(10);
  R_PORT4->PODR &= ~(1 << BIT_TRIG);

  // get echo
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 20000);
  if (duration == 0) return 999;
  
  // speed of sound = 343 m/s = 0.0343 cm/µs, divide by 2 for round trip
  return (duration * 0.0343) / 2;
}


int ldr_smoothing(void)
{
  static float filtered = 0;
  // RC filter type of thing keep 0.7 of old 0.3 of new measurement
  filtered = filtered * 0.7 + analogRead(PIN_LDR) * 0.3;

  return (int)filtered;
}
