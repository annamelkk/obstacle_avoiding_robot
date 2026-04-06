// ------------------ DEFINITIONS ------------------------


// H-Bridge
#define enA 	3
#define enB 	6
#define in1   2
#define in2 	4 
#define in3 	5
#define in4   7

 
// Sound sensor




// Set the speed (0 = off and 255 = max speed)
const int dead_zone = 245;
 
void setup() {
//  Serial.begin(9600);
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

  go_forward( dead_zone+10 );
  delay(1000);

  stop_all();
  delay(1000);

  go_backward( dead_zone+10 );
  delay(1000);

  stop_all();
  delay(1000);

  go_right( dead_zone+10, dead_zone);
  delay(1000);

  go_left( dead_zone+10, dead_zone);
  delay(1000);

}
 

// ---------------- FUNCTIONS --------------------

// DIRECTIONS

void go_forward(int speed) {
  
  Serial.println("Forward");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, speed); 
  analogWrite(enB, speed);
}

void go_backward(int speed) {
  
  Serial.println("Dackward");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, speed); 
  analogWrite(enB, speed);

}


void go_right(int speed1, int speed2) {
  
 // Serial.println("Right");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, speed1); 
  analogWrite(enB, speed2);
}


void go_left(int speed1, int speed2) {

 // Serial.println("Left");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, speed1); 
  analogWrite(enB, speed2);

}


void stop_all() {

 // Serial.println("Stop All");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

/*
 * // or we can use
 * analogWrite(enA, 0);
 * analogWrite(enB, 0);
 */

}



// SHAPES

void go_round_clockwise(int speed) {
  
 // Serial.println("Doing a circle!");

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set the motor speed
  analogWrite(enA, speed); 
  analogWrite(enB, (speed-100));
}


void right_triangle(int speed) {
  
  // Serial.println("Triangle");

  go_forward(speed);
  delay(1000);
  go_right(speed, speed-10);
  delay(400);
  go_forward(speed);
  delay(1000);
  go_right(speed, speed-10);
  delay(600);
  go_forward(speed);
}


void do_rectangle(int speed) {
  
  // Serial.println("Rectangle");

  go_forward(speed);
  delay(1000);
  go_right(speed, speed-10);
  delay(500);
  go_forward(speed);
  delay(1000);
  go_right(speed, speed-10);
  delay(500);
  go_forward(speed);
  delay(1000);
  go_right(speed, speed-10);
  delay(500);
  go_forward(speed);
  delay(1000);
}
