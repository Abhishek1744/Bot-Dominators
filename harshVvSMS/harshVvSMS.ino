// Motor Pin Definitions
#define IN1 11  // Left Motor IN1
#define IN2 12  // Left Motor IN2
#define IN3 8  // Right Motor IN1
#define IN4 9  // Right Motor IN2
#define ENA 5  // Left Motor Enable Pin
#define ENB 10 // Right Motor Enable Pin

// Define motor control pins
//const int in1 = 11;
//const int in2 = 12;
//const int pwmA = 5;
//const int in3 = 8;
//const int in4 = 9;
//const int pwmB = 10;
// IR Sensor Connections
#define ir1 A0
#define ir2 A1
#define ir3 A2 // This is your s3 sensor
#define ir4 A3
#define ir5 A4

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set IR sensor pins as inputs
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
}

// Motor control functions
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void moveForward() {
  Serial.println("Moving Forward");
  setMotorSpeed(255, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  Serial.println("Turning Right");
  setMotorSpeed(255, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  Serial.println("Turning Left");
  setMotorSpeed(255, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  setMotorSpeed(0, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeftFull() {
  Serial.println("Performing Full Left Turn");
  setMotorSpeed(255, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(10); // Adjust duration for a full 90-degree turn
}

void turnRightFull() {
  Serial.println("Performing Full Right Turn");
  setMotorSpeed(255, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(10); // Adjust duration for a full 90-degree turn
}

void handleWhiteLine() {
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);

  // Exit early if the center sensor is active or all sensors are off
  if (s3 == 1 || (s1 == 0 && s2 == 0 && s4 == 0 && s5 == 0)) {
    return; 
  }

  // Conditions for movement
  if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0) {
    moveForward(); // Only s3 detects the line
  } else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    moveForward(); // All sensors on the line
  } else if (s1 == 0 && s2 == 1 && s3 == 1) {
    turnLeft(); // Slight turn to the right
  } else if (s1 == 1 && s2 == 0 && s3 == 1) {
    turnLeft(); // Slight turn to the left
  } else if ((s1 == 0 && s2 == 0 && s4 == 1) || 
             (s1 == 0 && s2 == 0 && s4 == 1 && s5 == 1) || 
             (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1) || 
             (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1)) {
    turnRightFull(); // Handle various full right turn conditions
  } else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    turnRightFull(); // Perform a full left turn
  } else if ((s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0) || 
             (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) ||
             (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0)) {
    turnLeftFull(); // Perform a full right turn
  } else if (s1 == 0 && s2 == 0 && s4 == 1 && s5 == 1) {
    turnRightFull(); // Perform a full left turn
  } else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) {
    turnLeftFull(); // Perform a full left turn
  } else {
    moveForward(); // Default action
  }
}

void handleBlackLine() {
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);
  
// Exit early if the center sensor is active or all sensors are off
  if (s3 == 0 || (s1 == 1 && s2 == 1 && s4 == 1 && s5 == 1)) {
    return; 
  }
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    moveForward(); // All sensors off the line
  } else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 1 && s5 == 1) {
    moveForward(); // s3 sensor on the line
  } else if ((s1 == 0 && s2 == 1) || (s1 == 0 && s2 == 0 && s3 == 1) ||
             (s1 == 1 && s2 == 0 && s3 == 1)) {
    turnLeft(); // Sensors detecting a turn to the left
  } else if ((s1 == 1 && s2 == 1 && s4 == 0 && s5 == 1) ||
             (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0)) {
    turnRight(); // Sensors detecting a turn to the right
  }
  
  // Perform full turns for specific conditions
  if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    turnLeftFull(); // Perform a full left turn
  } else if ((s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0) || 
             (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) ||
             (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0)) {
    turnRightFull(); // Perform a full right turn
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1) {
    turnLeftFull(); // Perform a full left turn
  } else if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1) {
    turnLeftFull(); // Perform a full left turn
  }

  // Continue moving forward if the middle sensors are active
  if (s2 == 1 && s3 == 0) {
    moveForward();
  }
  
  delay(10); // Adjust the delay as needed
}

// Main loop
  void loop() {
  handleWhiteLine();
  handleBlackLine();
}
