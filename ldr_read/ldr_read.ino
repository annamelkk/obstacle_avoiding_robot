#define LDR_PIN A0

void setup() {
  
  Serial.begin(9600);

  pinMode(A0, INPUT);

}

void loop() {

  int level = analogRead(LDR_PIN);

  Serial.println(level);


  delay(1000);
}
