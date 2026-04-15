// ====================== DEFINITIONS ======================

#define PIN_ENA   3
#define PIN_ENB   6
#define BIT_IN1   4   // D2 - P104
#define BIT_IN2   6   // D4 - P106
#define BIT_IN3   7   // D5 - P107
#define BIT_IN4   12  // D7 - P112

#define PIN_ECHO          10
#define PIN_TRIG          11
#define BIT_TRIG          11  // D11 - P411
#define BIT_ECHO          3   // D10 - P103
#define OBSTACLE_DISTANCE 20  // cm

#define PIN_LDR   A0
#define BLACK     495
#define WHITE     780
#define THRESHOLD ((BLACK + WHITE)/2)

#define DEADBAND      5
#define DRIVE_SPEED   140 

// PID values
#define k_p 0.6
#define k_d 0.2
#define k_i 0.01 // Small integral to correct long-term drift

// Timing
#define PID_INTERVAL_MS 10

// ====================== GLOBALS =========================

float error = 0;
float prev_error = 0;
float integral = 0;
unsigned long last_pid_time = 0;

// ====================== SETUP ===========================

void setup() {
  Serial.begin(9600);

  // Set Output Directions using PDR
  R_PORT1->PDR |= (1 << BIT_IN1) | (1 << BIT_IN2) | (1 << BIT_IN3) | (1 << BIT_IN4);
  R_PORT4->PDR |= (1 << BIT_TRIG);
  R_PORT1->PDR &= ~(1 << BIT_ECHO);

  // Initialize pins to LOW using PCNTR3 (Atomic Clear)
  // Bits 16-31 of PCNTR3 clear the corresponding pins 0-15
  R_PORT1->PCNTR3 = (1 << (BIT_IN1 + 16)) | (1 << (BIT_IN2 + 16)) | 
                    (1 << (BIT_IN3 + 16)) | (1 << (BIT_IN4 + 16));
  R_PORT4->PCNTR3 = (1 << (BIT_TRIG + 16));

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  delay(1000);
}

// ====================== LOOP ============================

void loop() {
  // 1. Safety Check (Ultrasonic)
  if (get_distance_cm() < OBSTACLE_DISTANCE) {
    stop();
    return; // Skip the rest of the loop
  }

  // 2. Fixed frequency PID execution
  unsigned long current_time = millis();
  if (current_time - last_pid_time >= PID_INTERVAL_MS) {
    int ldr = ldr_smoothing();
    PID_CTRL(ldr);
    last_pid_time = current_time;
  }
}

// ====================== CONTROL FUNCTIONS ===============

void PID_CTRL(int ldr) {
  error = THRESHOLD - ldr;
  
  // Derivative: Change in error over time
  float derivative = error - prev_error;
  
  // Integral: Accumulated error over time
  integral += error;
  
  // Constrain integral to prevent "windup"
  if (integral > 500) integral = 500;
  if (integral < -500) integral = -500;

  float control = (k_p * error) + (k_i * integral) + (k_d * derivative);

  // Lost line recovery: if error is huge and not changing, force a spin
  if (abs(error) > 150 && abs(derivative) < 2) {
     control = (prev_error > 0) ? 120 : -120;
  }

  if (abs(control) < DEADBAND) control = 0;

  int left_speed  = DRIVE_SPEED + (int)control;
  int right_speed = DRIVE_SPEED - (int)control;

  drive(left_speed, right_speed);
  
  prev_error = error;
}

// ====================== MOTOR HELPERS ===================

// Renesas PCNTR3: Lower 16 bits SET, Upper 16 bits CLEAR
void set_right_motor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    R_PORT1->PCNTR3 = (1 << BIT_IN1);           // Set IN1 High
    R_PORT1->PCNTR3 = (1 << (BIT_IN2 + 16));    // Set IN2 Low
    analogWrite(PIN_ENA, speed);
  } else if (speed < 0) {
    R_PORT1->PCNTR3 = (1 << (BIT_IN1 + 16));    // Set IN1 Low
    R_PORT1->PCNTR3 = (1 << BIT_IN2);           // Set IN2 High
    analogWrite(PIN_ENA, -speed);
  } else {
    analogWrite(PIN_ENA, 0);
  }
}

void set_left_motor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    R_PORT1->PCNTR3 = (1 << BIT_IN3);           // Set IN3 High
    R_PORT1->PCNTR3 = (1 << (BIT_IN4 + 16));    // Set IN4 Low
    analogWrite(PIN_ENB, speed);
  } else if (speed < 0) {
    R_PORT1->PCNTR3 = (1 << (BIT_IN3 + 16));    // Set IN3 Low
    R_PORT1->PCNTR3 = (1 << BIT_IN4);           // Set IN4 High
    analogWrite(PIN_ENB, -speed);
  } else {
    analogWrite(PIN_ENB, 0);
  }
}

void drive(int l, int r) {
  set_left_motor(l);
  set_right_motor(r);
}

void stop() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  R_PORT1->PCNTR3 = (1 << (BIT_IN1 + 16)) | (1 << (BIT_IN2 + 16)) | 
                    (1 << (BIT_IN3 + 16)) | (1 << (BIT_IN4 + 16));
}

// ====================== SENSOR HELPERS ==================

int get_distance_cm() {
  R_PORT4->PCNTR3 = (1 << (BIT_TRIG + 16)); // Trig Low
  delayMicroseconds(2);
  R_PORT4->PCNTR3 = (1 << BIT_TRIG);        // Trig High
  delayMicroseconds(10);
  R_PORT4->PCNTR3 = (1 << (BIT_TRIG + 16)); // Trig Low

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 15000); // 15ms timeout (~2.5 meters)
  if (duration == 0) return 999;
  return (duration * 0.0343) / 2;
}

int ldr_smoothing(void) {
  static float filtered = THRESHOLD; 
  filtered = (filtered * 0.8) + (analogRead(PIN_LDR) * 0.2);
  return (int)filtered;
}
