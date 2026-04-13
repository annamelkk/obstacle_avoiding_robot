// ====================== DEFINITIONS ======================
#define PIN_ENA   3
#define PIN_ENB   6
#define BIT_IN1   4
#define BIT_IN2   6
#define BIT_IN3   7
#define BIT_IN4   12

#define PIN_LDR       A0
#define THRESHOLD     310
#define SEARCH_SPEED  100
#define DRIVE_SPEED   150

// ====================== GLOBALS =========================
int last_turn = 1;
float Kp = 0.2;

// ====================== SETUP ===========================
void setup()
{
  Serial.begin(9600);

  R_PORT1->PDR  |= (1 << BIT_IN1) | (1 << BIT_IN2) |
                   (1 << BIT_IN3) | (1 << BIT_IN4);

  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                     (1 << BIT_IN3) | (1 << BIT_IN4));

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  delay(2000);
}

// ====================== LOOP ============================
void loop()
{
  int ldr = analogRead(PIN_LDR);
  Serial.println(ldr);

  P_CTRL(ldr);
}

// ====================== CONTROL =========================

void P_CTRL(int ldr)
{
  if (ldr < THRESHOLD)
  {
    int error = ldr - THRESHOLD;

    if (error > 0)
      last_turn = 1;
    else
      last_turn = -1;

    track_correction(error);
  }
  else
  {
    search_line();
  }
}

// ====================== SEARCH ==========================

void search_line()
{
  static unsigned long last_switch = 0;
  static int direction = 1;

  // switch direction every 400 ms
  if (millis() - last_switch > 400)
  {
    direction *= -1;
    last_switch = millis();
  }

  if (direction == 1)
  {
    // curve right
    drive(60, SEARCH_SPEED);
  }
  else
  {
    // curve left
    drive(SEARCH_SPEED, 60);
  }
}
// ====================== TRACKING ========================

void track_correction(int error)
{
  int correction = Kp * error;

  int right_speed = constrain(DRIVE_SPEED + correction, 80, 255);
  int left_speed  = constrain(DRIVE_SPEED - correction, 80, 255);

  drive(right_speed, left_speed);
}

// ====================== MOTOR ===========================

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
  else
  {
    analogWrite(PIN_ENA, 0);
  }
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
  else
  {
    analogWrite(PIN_ENB, 0);
  }
}

// ✅ FINAL CORRECT SWAP
void drive(int right_speed, int left_speed)
{
  set_right_motor(-right_speed);
  set_left_motor(-left_speed);
}

void stop()
{
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}
