#define enA 	3
#define enB 	6
#define in1   2
#define in2 	4 
#define in3 	5
#define in4   7

 
// Set the speed (0 = off and 255 = max speed)
const int motorSpeed = 255;
 
void setup() {
  Serial.begin(9600);
  // motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 
  // turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
 
void loop() {
/*
  // Go forwards
  go_forward();
  delay(3000);

  // Stop
  stop_all();
  delay(3000);
 
  // Go backwards
  go_backward();
  delay(3000);

  stop_all();
  delay(3000);
 
  go_round_clockwise();
  delay(3000);

  stop_all();
  delay(3000);

*/
  right_triangle();
  delay(3000);

  stop_all();
  delay(3000);

}
 

// ---------------- FUNCTIONS --------------------

void go_forward() {
  
  Serial.println("Forward");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed);
}

void go_backward() {
  
  Serial.println("Dackward");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed);

}


void go_right() {
  
  Serial.println("Right");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed);
}


void go_left() {

  Serial.println("Left");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed);

}


void stop_all() {

  Serial.println("Stop All");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed);
}


void go_round_clockwise() {
  
  Serial.println("Doing a circle!");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, motorSpeed); 
  analogWrite(enB, (motorSpeed-100));
}


void right_triangle() {
  
  Serial.println("Triangle");

  go_forward();
  delay(1000);
  go_right();
  delay(400);
  go_forward();
  delay(1000);
  go_right();
  delay(600);
  go_forward();
}
