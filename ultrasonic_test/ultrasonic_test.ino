#define PIN_ECHO  10
#define PIN_TRIG  11

void setup() {
  Serial.begin(9600);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 20000);
  int distance = (duration * 0.0343) / 2;
  
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(200);
}
