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
#define BLACK         525 
#define THRESHOLD     610
#define WHITE         725
#define SEARCH_SPEED  100
#define DRIVE_SPEED   130

#define k_p           0.1
#define k_i           0.1
#define k_d           0.2

// ====================== GLOBALS =========================
unsigned long last_sensor_time  = 0;
int           cached_distance   = 999;

bool          was_on_line       = 1;
int           last_turn         = 1;   // 1 = spin right, -1 = spin left
float         error             = 0;
float         prev_error        = 0;
float         integral          = 0;
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

  delay(2000);
  last_sensor_time = millis();
}

// ====================== LOOP ============================
void loop()
{
  /*
  // read distance
  if (millis() - last_sensor_time >= 100)
  {
    cached_distance  = get_distance_cm();
    last_sensor_time = millis();
  }

  if (cached_distance < OBSTACLE_DISTANCE)
  {
    stop();
  }
  else
  {
  */
    // read ldr
    int ldr = analogRead(PIN_LDR);
    Serial.print("LDR value: "); Serial.println(ldr);

    ON_OFF_CTRL(ldr);   // on/off control function with fixed PWM
    // P_CTRL(ldr);     // p-control function where motor PWM is proportional to error
  //}
  
  Serial.print("LDR=");
  Serial.print(analogRead(PIN_LDR));
  
  Serial.print(" | TH=");
  Serial.print(THRESHOLD);
  
  Serial.print(" | ON=");
  Serial.print(is_on_line());

  Serial.print(" | LT=");
  Serial.println(last_turn);
}

// ====================== CONTROL FUNCTIONS ===============

void ON_OFF_CTRL(int ldr)
{
  if (is_on_line())          // case black 
  {
    drive(DRIVE_SPEED, DRIVE_SPEED);
    was_on_line = 1;
  }
  else                      // case white 
    search_line();
}

void P_CTRL(int ldr)
{
  if (is_on_line())          // case black 
  {
    drive(DRIVE_SPEED, DRIVE_SPEED);
    
    error = 0;
    integral = 0;
    prev_error = 0;
  }
  else                      // case white 
    pid_search();
}

// ====================== LINE HELPERS ====================

void search_line()
{
  if (was_on_line)
  {
    last_turn *= -1;
    was_on_line = 0;
  }
  if (last_turn == 1)
  {
    Serial.println("turining left");
    drive(SEARCH_SPEED, DRIVE_SPEED);
  }
  else
  {
    Serial.println("turning right");
    drive(DRIVE_SPEED, SEARCH_SPEED);
  }
}


void pid_search(int sensor_val)
{
  error += last_turn;

  integral += error;
  
  float derivative = error - prev_error;

  float correction = (k_p * error) + (k_i * integral) + (k_d * derivative);

  int left_speed  = DRIVE_SPEED + correction;
  int right_speed = DRIVE_SPEED - correction;

  left_speed  = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  drive(left_speed, right_speed);

  prev_error = error;
}

// ====================== MOTOR HELPERS ===================

void set_right_motor(int speed)
{
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

bool is_obstacle() { return cached_distance < OBSTACLE_DISTANCE; }
bool is_on_line()  { return analogRead(PIN_LDR) < THRESHOLD; }
