#include <Servo.h>

#define TRIG A0
#define ECHO A1
#define OBSTACLE_DISTANCE 5
#define SONAR_INTERVAL 60 // Increased slightly for echo stability

#define BIG 3
#define SMALL 5

Servo big;
Servo small;

unsigned long last_sonar = 0;
int DISTANCE = 999;

void setup() {
  Serial.begin(9600);
  big.attach(BIG, 544, 2500);
  small.attach(SMALL, 544, 2400);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  unsigned long now = millis();
  
  if (now - last_sonar > SONAR_INTERVAL) {
    last_sonar = now;
    DISTANCE = get_distance();
    
    Serial.print("Distance: ");
    Serial.print(DISTANCE);
    Serial.println(" cm");
  }

  if (DISTANCE < OBSTACLE_DISTANCE) {
    Serial.println("!!! Obstacle detected !!!");
    avoidance_sequence();
  } else {
    // Neutral position
    big.write(0);
    small.write(0);
    Serial.println("Position neutral");
  }
}

void avoidance_sequence() {
  big.write(180);
  delay(800);
  small.write(180);
  delay(1000);
}

int get_distance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // pulseIn returns duration in microseconds
  unsigned long duration = pulseIn(ECHO, HIGH, 25000); 
  
  if (duration == 0) {
    return 999; // Return out-of-range value if no echo
  }
  
  // Speed of sound calculation
  int d = (duration * 0.0343) / 2;
  return d;
}
