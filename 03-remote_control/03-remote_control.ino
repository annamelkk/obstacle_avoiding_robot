#include "thing_properties.h"
#include "Arduino_LED_Matrix.h"

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
#define ECHO              10
#define TRIG              11
#define OBSTACLE_DISTANCE 35
#define SONAR_INTERVAL 50

// line sensor
#define LDR       A0
#define BLACK     680
#define WHITE     1023
#define GRAY      900
#define THRESHOLD 900

#define DRIVE_SPEED   45

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



// ====================== GLOBALS ==========================

float error = 0;
float prev_error = 0;
float integral = 0;
float derivative = 0;

unsigned long last_pid_time = 0;
unsigned long last_sonar = 0;

ArduinoLEDMatrix matrix;

uint8_t heart[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,1,1,1,1,0,0,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0}
};

// ====================== SETUP ============================

void setup() 
{
  Serial.begin(9600);
  delay(1500);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  matrix.begin();


  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LDR, INPUT);

  matrix.loadSequence(LEDMATRIX_ANIMATION_WIFI_SEARCH);
  matrix.play(true);

  stop();
}


// ====================== LOOP =============================

void loop() 
{
  ArduinoCloud.update();
  handleMatrix();
  Serial.println(analogRead(LDR));
  unsigned long now = millis();
  // update distance for dashboard
  if (now - last_sonar > SONAR_INTERVAL)
  {
    last_sonar = now;
    DISTANCE = get_distance_cm();
  }

  // ================= REMOTE CONTROL =================
  if (!AUTO_ON)
  {
    remote_control();
    return;
  }

  // ================= AUTO MODE ======================
  if (DISTANCE < OBSTACLE_DISTANCE)
  {
    stop();
    return;
  }

  int ldr = analogRead(LDR);
  Serial.println(ldr);
#if CONTROL_MODE == CTRL_ON_OFF
    ON_OFF_CTRL(ldr);
#elif CONTROL_MODE == CTRL_P
    P_CTRL(ldr);
#elif CONTROL_MODE == CTRL_PID
    PID_CTRL(ldr);
#endif

}


// ====================== REMOTE CONTROL ==================

void remote_control()
{
  int left  = map(LEFT_MOTOR_SPEED,  -100, 100, -255, 255);
  int right = map(RIGHT_MOTOR_SPEED, -100, 100, -255, 255);

  // dead zone
  if (abs(left) < 10) left = 0;
  if (abs(right) < 10) right = 0;

  drive(left, right);
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
  Serial.println(error);
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

int get_distance_cm() 
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


// ================= HEART ANIMATION ======================

void handleMatrix()
{
  static bool connectedShown = false;

  if (ArduinoCloud.connected())
  {
    if (!connectedShown)
    {
      matrix.renderBitmap(heart, 12, 8);
      matrix.play(true);
      connectedShown = true;
    }
  }
  else
  {
    if (connectedShown)
    {
      matrix.loadSequence(LEDMATRIX_ANIMATION_WIFI_SEARCH);
      matrix.play(true);
      connectedShown = false;
    }
  }
}



// ================= CLOUD CALLBACKS ======================

void onLEFTMOTORSPEEDChange() 
{
  if (!AUTO_ON) remote_control();
}

void onRIGHTMOTORSPEEDChange() 
{
  if (!AUTO_ON) remote_control();
}

void onAUTOONChange() 
{
  stop();

  integral = 0;
  prev_error = 0;
  last_pid_time = millis();
}
