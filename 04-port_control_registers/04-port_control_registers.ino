// ------------------ DEFINITIONS ------------------------
// Motor and H-bridge
#define PIN_ENA   3
#define PIN_ENB   6
#define BIT_IN1   4   // D2 - P104
#define BIT_IN2   6   // D4 - P106
#define BIT_IN3   7   // D5 - P107
#define BIT_IN4   12  // D7 - P112


// Ultrasonic
#define PIN_ECHO  10
#define PIN_TRIG  11
#define BIT_TRIG  11  // D11 - P411
#define BIT_ECHO  3   // D10 - P103
#define OBSTACLE_DISTANCE 20

unsigned long last_sensor_time = 0;
int           cached_distance = 999;


// LDR
#define PIN_LDR   A0
#define BLACK         290
#define THRESHOLD     310
#define WHITE         405
#define SEARCH_SPEED  100
#define DRIVE_SPEED   130

int   last_turn = 1; // 1 spin right, -1 spin left
float Kp = 0.1;
int   last_error = 0;


void setup()
{
  Serial.begin(9600);

  // Motors - all on PORT1
  R_PORT1->PDR |= (1 << BIT_IN1) | (1 << BIT_IN2) |
                  (1 << BIT_IN3) | (1 << BIT_IN4);
  R_PORT1->PODR &= ~((1 << BIT_IN1) | (1 << BIT_IN2) |
                     (1 << BIT_IN3) | (1 << BIT_IN4));

  // Sensor
  R_PORT4->PDR  |=  (1 << BIT_TRIG); // TRIG output - PORT4 bit 11
  R_PORT1->PDR  &= ~(1 << BIT_ECHO); // ECHO input  - PORT1 bit 3
  R_PORT4->PODR &= ~(1 << BIT_TRIG); // TRIG starts LOW

  // PWM pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);


  delay(2000); // 2 seconds to place robot before sensor starts
  last_sensor_time = millis();

}

void  loop()
{
  Serial.println(analogRead(PIN_LDR));
  int ldr_value = analogRead(PIN_LDR);

  if (ldr_value < THRESHOLD)
  {
    last_error = ldr_value - THRESHOLD;
    track_correction(ldr_value);
  }
  else
    search_line();


  /*
  if (millis() - last_sensor_time >= 100)
  {
    cached_distance = get_distance_cm();
    Serial.print("Distance: ");
    Serial.println(cached_distance);
    last_sensor_time = millis();
  }

  if (cached_distance < OBSTACLE_DISTANCE)
    stop();
  else
    drive(150, 150);
    */
}

void  set_right_motor(int speed)
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

void  set_left_motor(int speed)
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

void  drive(int right_speed, int left_speed)
{
  set_right_motor(right_speed);
  set_left_motor(left_speed);
}

void  stop()
{
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  R_PORT1->PODR &= ~(1 << BIT_IN1);
  R_PORT1->PODR &= ~(1 << BIT_IN2);
  R_PORT1->PODR &= ~(1 << BIT_IN3);
  R_PORT1->PODR &= ~(1 << BIT_IN4);
}

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

bool  is_obstacle()
{
  return cached_distance < OBSTACLE_DISTANCE;
}


bool  is_on_line()
{
  return analogRead(PIN_LDR) < THRESHOLD; // below threshold means black
}


void  search_line()
{
    if (last_error < 1)
      drive(SEARCH_SPEED, -SEARCH_SPEED); // spin right
    else
      drive(-SEARCH_SPEED, SEARCH_SPEED); // spin left
}

void  track_correction(int sensor_val)
{
  int error = sensor_val - THRESHOLD;
  int correction = Kp * error;

  Serial.print("sensor: "); Serial.print(sensor_val);
  Serial.print("  error: "); Serial.println(error);
  int right_corrected = constrain(DRIVE_SPEED - correction, 60, 255);
  int left_corrected = constrain(DRIVE_SPEED + correction, 60, 255);

  drive(right_corrected, left_corrected);
}
