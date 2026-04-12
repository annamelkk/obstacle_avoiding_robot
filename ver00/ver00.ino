#define DZ 85
#define ENA 3
#define IN1 2
#define IN2 4
#define ENB 6
#define IN3 7
#define IN4 5
#define sensorPin A0
#define trigPin 11
#define echoPin 10

// calibration
int bwCutoff = 85;   
int margin = 5;    

int slowLeft  = 150;
int slowRight = 135;
int obstacleDistance = 15;

unsigned long lastAvoidTime = 0;
unsigned long avoidCooldown = 2000;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void forwardSlow() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, slowLeft);
  analogWrite(ENB, slowRight);
}

void turnLeftSlow() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, slowLeft);
  analogWrite(ENB, 0);
}

void turnRightSlow() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 0);
  analogWrite(ENB, slowRight);
}

void stopBot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

//ultrasonic
long getDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return 999;

  long distance = duration * 0.034 / 2;
  return distance;
}

bool obstacleDetected() {
  if (millis() - lastAvoidTime < avoidCooldown) return false;

  long dist = getDistanceCM();

  Serial.print("Distance: ");
  Serial.println(dist);

  if (dist <= obstacleDistance) {
    return true;
  } else {
    return false;
  }
}

// obstacle avoidance 
void avoidObstacle() {
  stopBot();
  delay(100);

  // turn left
  turnLeftSlow();
  delay(400);

  // go forward
  forwardSlow();
  delay(500);

  // turn right
  turnRightSlow();
  delay(400);

  stopBot();
  delay(100);

  lastAvoidTime = millis();
}

void loop() {
  int ldr = analogRead(sensorPin);

  Serial.print("LDR: ");
  Serial.println(ldr);

  // obstacle handling first
  if (obstacleDetected()) {
    avoidObstacle();
    return;
  }

  //ldr
  if (ldr < bwCutoff - margin) {
    // black
    turnLeftSlow();

  } else if (ldr > bwCutoff + margin) {
    //white 
    turnRightSlow();

  } else {
    //grey
    forwardSlow();
  }

  delay(10);
}
