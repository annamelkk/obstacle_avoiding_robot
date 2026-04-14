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
#define OBSTACLE_DISTANCE 20

// LDR
#define PIN_LDR       A0
#define BLACK         495
#define DEADBAND      8
#define THRESHOLD     500
#define WHITE         730


#define LINE_LOST_W   700
#define LINE_LOST_B   400


#define SEARCH_SPEED  110
#define DRIVE_SPEED   255
#define RECOVER_SPEED 110

#define k_p           0.4
#define k_i           0.02
#define k_d           0.1

// ====================== GLOBALS =========================

float error = 0.0;
float prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;

float filtered_sensor = 0.0;
float alpha = 0.35;

unsigned long prev_time = 0;
int last_turn = 1;

unsigned int last_sensor_time = 0;
int           cached_distance = 999;


// ====================== SETUP ===========================
void setup()
{
  Serial.begin(9600);

  // Motor direction pins – all on PORT1
  R_PORT1->PDR  |=  (1 << BIT_IN1) | (1 << BIT_IN2) |
                    (1 << BIT_IN3) | (1 << BIT_IN4);
  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                     (1 << BIT_IN3) | (1 << BIT_IN4));

  // Ultrasonic sensor
  R_PORT4->PDR  |=  (1 << BIT_TRIG); // TRIG output – PORT4 bit 11
  R_PORT1->PDR  &= ~(1 << BIT_ECHO); // ECHO input  – PORT1 bit 3
  R_PORT4->PODR &= ~(1 << BIT_TRIG); // TRIG starts LOW

  // PWM enable pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  int first_read = analogRead(PIN_LDR);
  filtered_sensor = first_read;

  prev_time = micros();
  last_sensor_time = micros();

  delay(2000);
  
}

// ====================== LOOP ============================
void loop()
{
  // read distance
  if (micros() - last_sensor_time >= 100)
  {
    cached_distance  = get_distance_cm();
    last_sensor_time = micros();
  }

  if (cached_distance < OBSTACLE_DISTANCE)
    stop();
  else
  {
    // read ldr
    int ldr = analogRead(PIN_LDR);
    Serial.print("LDR value: "); Serial.println(ldr);

    PID_CTRL();     // p-control function where motor PWM is proportional to error
  }
}

// ====================== CONTROL FUNCTIONS ===============
void PID_CTRL() {
  int sensor_val = read_filtered_sensor();

  // Time step in seconds
  unsigned long now = micros();
  float dt = (now - prev_time) / 1000000.0;
  prev_time = now;

  // Prevent bad derivative spikes
  if (dt <= 0.0) dt = 0.001;
  if (dt > 0.05) dt = 0.05;

  // Detect probable line loss
  if (sensor_val >= LINE_LOST_W || sensor_val <= LINE_LOST_B) {
    recover_line();
    return;
  }

  // Error = target edge - actual reading
  error = (float)(THRESHOLD - sensor_val);

  // Deadband to reduce zigzag
  if (abs((int)error) <= DEADBAND) {
    error = 0.0;
  }

  // Integral with anti-windup
  integral += error * dt;
  if (integral > 40.0) integral = 40.0;
  if (integral < -40.0) integral = -40.0;

  // Derivative
  derivative = (error - prev_error) / dt;

  // PID output
  float correction = k_p * error + k_i * integral + k_d * derivative;

  // Remember last useful direction
  if (error > 0) last_turn = 1;
  if (error < 0) last_turn = -1;

  // Slow down on sharper turns
  int current_base = DRIVE_SPEED;


  if (abs((int)error) > 35) {
    current_base = SEARCH_SPEED;
  }

  int right_speed = current_base - correction;
  int left_speed  = current_base + correction;

  drive(right_speed, left_speed);

  prev_error = error;
}

// ====================== LINE HELPERS ====================


void recover_line()
{
  if (last_turn == 1)
    drive(-RECOVER_SPEED, RECOVER_SPEED);
  else
    drive(RECOVER_SPEED, -RECOVER_SPEED);
}

bool is_on_line()  { return analogRead(PIN_LDR) < THRESHOLD; }
// ====================== MOTOR HELPERS ===================

void set_right_motor(int speed)
{
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;
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

void drive(int right_speed, int left_speed)
{
  set_right_motor(right_speed);
  set_left_motor(left_speed);
}

void stop()
{
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  R_PORT1->PODR &= ~(1 << BIT_IN1);
  R_PORT1->PODR &= ~(1 << BIT_IN2);
  R_PORT1->PODR &= ~(1 << BIT_IN3);
  R_PORT1->PODR &= ~(1 << BIT_IN4);
}

// ====================== SENSOR HELPERS ==================

int get_distance_cm()
{
  R_PORT4->PODR &= ~(1 << BIT_TRIG);
  delayMicroseconds(2);
  R_PORT4->PODR |=  (1 << BIT_TRIG);
  delayMicroseconds(10);
  R_PORT4->PODR &= ~(1 << BIT_TRIG);
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 20000);
  if (duration == 0) return 999;
  return (duration * 0.0343) / 2;
}

int read_filtered_sensor()
{
  int raw = analogRead(PIN_LDR);
  filtered_sensor = alpha * raw + (1.0 - alpha) * filtered_sensor;
  return (int)filtered_sensor;
}


