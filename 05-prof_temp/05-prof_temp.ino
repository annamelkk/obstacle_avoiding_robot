// ====================== DEFINITIONS ======================
// Motor and H-bridge
#define DZ        100
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
#define PIN_LDR   A0
#define BLACK     290
#define WHITE     405
#define GREY      350  // ~347

// Speeds
#define DRIVE_SPEED   130
#define SEARCH_SPEED  100

// ====================== GLOBALS =========================
unsigned long last_sensor_time = 0;
int           cached_distance  = 999;
int           ldr              = 0;

// ====================== SETUP ===========================
void setup()
{
  Serial.begin(9600);

  // Motor direction pins – all on PORT1
  R_PORT1->PDR  |=  (1 << BIT_IN1) | (1 << BIT_IN2) | // all INs as output
                    (1 << BIT_IN3) | (1 << BIT_IN4);

  R_PORT1->PODR &=  ~((1 << BIT_IN1) | (1 << BIT_IN2) | // start LOW
                     (1 << BIT_IN3) | (1 << BIT_IN4));

  // Ultrasonic sensor
  R_PORT4->PDR  |=  (1 << BIT_TRIG); // TRIG output – PORT4 bit 11
  R_PORT1->PDR  &= ~(1 << BIT_ECHO); // ECHO input  – PORT1 bit 3
  R_PORT4->PODR &= ~(1 << BIT_TRIG); // TRIG starts LOW

  // PWM enable pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  delay(2000); // to put robot into position
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
  */
    // read ldr
    ldr = analogRead(PIN_LDR);
    Serial.print("LDR: "); Serial.println(ldr);

   // ON_OFF_CTRL();   // on/off control function with fixed PWM
   P_CTRL();     // p-control function where motor PWM is proportional to error
  //}
}

// ====================== CONTROL FUNCTIONS ===============

void ON_OFF_CTRL()
{
  if (ldr > GREY + 10)           // case white — veer right
  {
    drive(DRIVE_SPEED, SEARCH_SPEED);
  }
  else if (ldr < GREY - 10)      // case black — veer left
  {
    drive(SEARCH_SPEED, DRIVE_SPEED);
  }
  else                           // case grey — on the line, drive straight
  {
    drive(DRIVE_SPEED, DRIVE_SPEED);
  }
}

void P_CTRL()
{
  // calculate error (positive = white side, negative = black side)
  int error = ldr - GREY;
  int correction = constrain((int)(0.3f * abs(error)), 0, 155); // 100+155=255 max

  if (error > 10)           // case white — veer right
  {
    drive(DZ + correction, DZ);
  }
  else if (error < -10)     // case black — veer left
  {
    drive(DZ, DZ + correction);
  }
  else                      // case grey — on the line, drive straight
  {
    drive(DZ, DZ);
  }
}

// ====================== MOTOR HELPERS ===================

void set_right_motor(int speed)
{
  if (speed > 0)
  {
    R_PORT1->PODR &= ~(1 << BIT_IN1);
    R_PORT1->PODR |=  (1 << BIT_IN2);
    analogWrite(PIN_ENA, speed);
  }
  else if (speed < 0)
  {
    R_PORT1->PODR |=  (1 << BIT_IN1);
    R_PORT1->PODR &= ~(1 << BIT_IN2);
    analogWrite(PIN_ENA, -speed);
  }
  else { analogWrite(PIN_ENA, 0); }
}

void set_left_motor(int speed)
{
  if (speed > 0)
  {
    R_PORT1->PODR &= ~(1 << BIT_IN3);
    R_PORT1->PODR |=  (1 << BIT_IN4);
    analogWrite(PIN_ENB, speed);
  }
  else if (speed < 0)
  {
    R_PORT1->PODR |=  (1 << BIT_IN3);
    R_PORT1->PODR &= ~(1 << BIT_IN4);
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
