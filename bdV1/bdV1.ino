// Define motor control pins
const int enB = 11; // Enable pin for right motor
const int in4 = 10; // Control pin for right motor backward
const int in3 = 9;  // Control pin for right motor forward
const int in2 = 6;  // Control pin for left motor backward
const int in1 = 5;  // Control pin for left motor forward
const int enA = 3;  // Enable pin for left motor

// Define sensor pins (using analog pins for digital input)
const int S1 = A1;   // Sensor 1
const int S2 = A2;   // Sensor 2
const int S3 = A3;   // Sensor 3 (center sensor)
const int S4 = A4;   // Sensor 4
const int S5 = A5;   // Sensor 5

void setup() {
  // Set motor pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  
  // Set sensor pins as inputs
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  
  Serial.begin(9600); // For debugging
}

void loop() {
  // Read sensor values (0 or 1)
  int valueS1 = digitalRead(S1);
  int valueS2 = digitalRead(S2);
  int valueS3 = digitalRead(S3);
  int valueS4 = digitalRead(S4);
  int valueS5 = digitalRead(S5);
  
  // Print sensor values for debugging
  Serial.print("S1: ");
  Serial.print(valueS1);
  Serial.print(" | S2: ");
  Serial.print(valueS2);
  Serial.print(" | S3: ");
  Serial.print(valueS3);
  Serial.print(" | S4: ");
  Serial.print(valueS4);
  Serial.print(" | S5: ");
  Serial.println(valueS5);
  
  // Line following logic
  if (valueS3 == HIGH && valueS4 == HIGH) {
      // Centered on the line
      moveForward();
  } 
  else if (valueS2 == HIGH && valueS3 == HIGH) {
      // Slightly left of center
      turnslightLeft();
  } 
  else if (valueS4 == HIGH && valueS5 == HIGH) {
      // Slightly right of center
      turnslightRight();
  } 
  else if (valueS1 == HIGH || (valueS1 == HIGH && valueS2 == HIGH && valueS3 == HIGH)) {
      // Far left
      turnsharpLeft();
  } 
  else if (valueS5 == HIGH || (valueS5 == HIGH && valueS4 == HIGH && valueS3 == HIGH)) {
      // Far right
      turnsharpRight();
  } 


}

// Function to move forward
void moveForward() {
  digitalWrite(in1, HIGH);  // Left motor forward
  digitalWrite(in2, LOW);   // Left motor backward
  digitalWrite(in3, HIGH);  // Right motor forward
  digitalWrite(in4, LOW);   // Right motor backward
  analogWrite(enA, 140);    // Enable left motor at full speed
  analogWrite(enB, 200);     // Enable right motor at full speed
}

// Function to turn right
void turnslightRight() {
  digitalWrite(in1, HIGH);  // Left motor forward
  digitalWrite(in2, LOW);   // Left motor backward
  digitalWrite(in3, LOW);   // Right motor forward
  digitalWrite(in4, LOW);   // Right motor backward
  analogWrite(enA, 100);    // Enable left motor at 100 speed
  analogWrite(enB, 0);      // Disable right motor
}

void turnsharpRight() {
  digitalWrite(in1, HIGH);  // Left motor forward
  digitalWrite(in2, LOW);   // Left motor backward
  digitalWrite(in3, LOW);   // Right motor forward
  digitalWrite(in4, HIGH);  // Right motor backward
  analogWrite(enA, 140);    // Enable left motor at full speed
  analogWrite(enB, 140);    // Disable right motor
}

// Function to turn left
void turnslightLeft() {
  digitalWrite(in1, HIGH);   // Left motor forward
  digitalWrite(in2, LOW);    // Left motor backward
  digitalWrite(in3, HIGH);   // Right motor forward
  digitalWrite(in4, LOW);    // Right motor backward
  analogWrite(enA, 0);       // Disable left motor
  analogWrite(enB, 100);     // Enable right motor at 100 speed
}

void turnsharpLeft() {
  digitalWrite(in1, HIGH);   // Left motor forward
  digitalWrite(in2, LOW);    // Left motor backward
  digitalWrite(in3, HIGH);   // Right motor forward
  digitalWrite(in4, LOW);    // Right motor backward
  analogWrite(enA, 140);     // Enable left motor at full speed
  analogWrite(enB, 140);     // Enable right motor at full speed
}

// Function to stop motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);      // Disable left motor
  analogWrite(enB, 0);      // Disable right motor
}