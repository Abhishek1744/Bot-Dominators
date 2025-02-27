// Pin definitions for IR sensors
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {2, 3, 4, 5, 6, 7, 8, 9}; // D2-D9

// Pin definitions for motor driver
const int ENA = 11;
const int ENB = 10;
const int IN1 = 13;
const int IN2 = 12;
const int IN3 = A0;
const int IN4 = A1;

// Robot performance profile - Critical settings for overall behavior
#define PROCESSING_FREQUENCY 200  // Hz (200 = 5ms cycle time)
#define RESPONSE_MODE 2           // 1=Smooth, 2=Responsive, 3=Aggressive
#define PREDICTIVE_TURNING true   // Enable/disable turn prediction
#define ENABLE_HYSTERESIS true    // Enable/disable turn hysteresis
#define SENSOR_READ_MODE 1        // 1=Fast digital, 2=Multi-sample digital, 3=Analog with threshold

// PID Constants - Tuned for specific response profiles
float KP = 0.85;  // Higher value for more aggressive response
float KI = 0.02;  // Small integral for eliminating steady-state error
float KD = 0.5;   // High derivative for anticipating turns

// Dynamic weight array for weighted average calculations - outer sensors have higher weights
int SENSOR_WEIGHTS[NUM_SENSORS] = {-100, -70, -40, -15, 15, 40, 70, 100};

// Motor speed constants - Adjusted for higher performance
const int BASE_SPEED = 80;          // Baseline speed (increased)
const int NORMAL_SPEED = 160;        // Speed for straight lines
const int SLIGHT_TURN_SPEED = 100;   // Speed for slight turns (reduced)
const int SHARP_TURN_SPEED = 60;     // Speed for sharp turns (reduced)
const int EXTREME_TURN_SPEED = 0;    // Complete stop of inner wheel for extreme turns
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;

// Turn enhancement factors
const int OUTER_WHEEL_BOOST = 60;        // Extra speed for outer wheel in turns
const int EXTREME_OUTER_BOOST = 75;      // Even more boost for extreme turns
const int SHARP_TURN_THRESHOLD = 50;     // Threshold for identifying sharp turns
const int PREDICTIVE_TURN_FACTOR = 15;   // How much to amplify predicted turns

// Variables for PID and line detection
float lastError = 0;
float integral = 0;
int sensorValues[NUM_SENSORS];
unsigned long lastLineTime = 0;
const unsigned long LINE_TIMEOUT = 80;  // Reduced timeout for faster recovery

// Turn prediction variables
float errorHistory[5] = {0, 0, 0, 0, 0};
int historyIndex = 0;
float errorDerivative = 0;
float secondDerivative = 0;

// Turn direction memory and tracking
int lastDirection = 0;  // 0=straight, 1=left, 2=right
int directionConfidence = 0;  // How confident we are in the current direction
int previousSensorPattern = 0;
unsigned long lastDirectionChangeTime = 0;

// High-speed pattern detection variables
const unsigned long PATTERN_MEMORY_DURATION = 50;  // ms to remember patterns
unsigned long lastPatterns[5] = {0, 0, 0, 0, 0};
int patternValues[5] = {0, 0, 0, 0, 0};
int patternIndex = 0;

// Turn detection thresholds
const int ALL_LINE = B11111111;
const int NO_LINE = B00000000;

// Timing variables for performance monitoring
unsigned long lastCycleTime = 0;
unsigned long cycleTimeTotal = 0;
int cycleCount = 0;

// Hysteresis parameters to prevent oscillation
const int DIRECTION_HYSTERESIS = 3;  // Prevents rapid direction changes
bool inSharpTurn = false;
unsigned long sharpTurnStartTime = 0;
const unsigned long MIN_SHARP_TURN_DURATION = 150;  // ms

// Global flag to determine if sensor readings need inversion
// (ensuring that the "line" is always interpreted as active, i.e. 1)
bool invertSensorReadings = false;

// Calibration routine using only the outer sensors at startup.
// Assumes that when the robot starts on the line, the outer sensors (indices 0, 1, 6, 7)
// are over the background.
void calibrateSensors() {
  Serial.println("Calibrating using outer sensors. Ensure outer sensors are over background.");
  const int numReadings = 100;
  unsigned long sum = 0;
  int sensorIndices[] = {0, 1, NUM_SENSORS - 2, NUM_SENSORS - 1};  // indices 0, 1, 6, 7
  int numOuterSensors = sizeof(sensorIndices) / sizeof(sensorIndices[0]);
  
  for (int j = 0; j < numReadings; j++) {
    for (int k = 0; k < numOuterSensors; k++) {
      int i = sensorIndices[k];
      sum += digitalRead(sensorPins[i]);
    }
    delay(10);
  }
  float average = sum / float(numReadings * numOuterSensors);
  Serial.print("Calibration average of outer sensors: ");
  Serial.println(average);
  
  // If the average is high, the background is white.
  // For a white background, a reflective sensor returns HIGH (1),
  // and since the robot is meant to run on a black line on white background,
  // we need to invert the readings (so that the black line, which normally gives 0, becomes 1).
  if (average > 0.5) {
    invertSensorReadings = true;
    Serial.println("Detected white background. Configuring for a black line on white background.");
  } else {
    invertSensorReadings = false;
    Serial.println("Detected black background. Configuring for a white line on black background.");
  }
}

// Dynamically update line type during operation.
// Reads raw values from the outer sensors and adjusts the inversion flag.
void dynamicUpdateLineType() {
  int sensorIndices[] = {0, 1, NUM_SENSORS - 2, NUM_SENSORS - 1};  // Outer sensors
  const int numOuterSensors = sizeof(sensorIndices) / sizeof(sensorIndices[0]);
  int sum = 0;
  for (int i = 0; i < numOuterSensors; i++) {
    sum += digitalRead(sensorPins[sensorIndices[i]]);
  }
  float average = sum / float(numOuterSensors);
  // For a white background, raw reading is HIGH (1).
  bool newInvert = (average > 0.5);
  if (newInvert != invertSensorReadings) {
    invertSensorReadings = newInvert;
    Serial.print("Dynamic line type update: ");
    if (invertSensorReadings) {
      Serial.println("White background detected. Adjusting for black line on white background.");
    } else {
      Serial.println("Black background detected. Adjusting for white line on black background.");
    }
  }
}

// Sensor reading functions (applying inversion if needed)
int readSensorsQuick() {
  int sensorByte = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = digitalRead(sensorPins[i]);
    if (invertSensorReadings) {
      val = !val;
    }
    sensorValues[i] = val;
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  return sensorByte;
}

int readSensorsMultiSample() {
  int sensorByte = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int votes = digitalRead(sensorPins[i]);
    delayMicroseconds(100);
    votes += digitalRead(sensorPins[i]);
    delayMicroseconds(100);
    votes += digitalRead(sensorPins[i]);
    int value = (votes >= 2) ? 1 : 0;
    if (invertSensorReadings) {
      value = !value;
    }
    sensorValues[i] = value;
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  return sensorByte;
}

int readSensorsAnalog() {
  int sensorByte = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(sensorPins[i]);
    int value = (reading < 500) ? 1 : 0;  // Adjust threshold as needed
    if (invertSensorReadings) {
      value = !value;
    }
    sensorValues[i] = value;
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  return sensorByte;
}

int readSensorArray() {
  switch (SENSOR_READ_MODE) {
    case 1: return readSensorsQuick();
    case 2: return readSensorsMultiSample();
    case 3: return readSensorsAnalog();
    default: return readSensorsQuick();
  }
}

// Enhanced position calculation using weighted sensors
float calculatePosition() {
  float weightedSum = 0;
  int activeCount = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) {
      weightedSum += SENSOR_WEIGHTS[i];
      activeCount++;
    }
  }
  
  if (activeCount == 0) return 0;
  
  return weightedSum / activeCount;
}

// Record sensor pattern with timestamp for pattern prediction
void recordPattern(int pattern) {
  patternValues[patternIndex] = pattern;
  lastPatterns[patternIndex] = millis();
  patternIndex = (patternIndex + 1) % 5;
}

// Check if a specific pattern was seen recently
bool wasPatternRecentlySeen(int pattern) {
  unsigned long currentTime = millis();
  for (int i = 0; i < 5; i++) {
    if (patternValues[i] == pattern && (currentTime - lastPatterns[i]) < PATTERN_MEMORY_DURATION) {
      return true;
    }
  }
  return false;
}

// Enhanced motor control with emergency brake capability
void setMotors(int leftSpeed, int rightSpeed, const char* state) {
  // Right motor control (treated as left in pin mapping)
  digitalWrite(IN3, rightSpeed >= 0 ? LOW : HIGH);
  digitalWrite(IN4, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(ENB, abs(rightSpeed));
  
  // Left motor control (treated as right in pin mapping)
  digitalWrite(IN1, leftSpeed >= 0 ? LOW : HIGH);
  digitalWrite(IN2, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(ENA, abs(leftSpeed));
  
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(" L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
}

// Emergency stop - rapidly halts the robot
void emergencyBrake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(50);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Advanced PID controller with turn prediction
float calculateAdvancedPID() {
  float position = calculatePosition();
  float error = position;
  
  errorHistory[historyIndex] = error;
  historyIndex = (historyIndex + 1) % 5;
  
  errorDerivative = error - lastError;
  
  int prevIndex = (historyIndex + 4) % 5;
  int prevPrevIndex = (historyIndex + 3) % 5;
  secondDerivative = (errorHistory[prevIndex] - errorHistory[prevPrevIndex]) - errorDerivative;
  
  integral = integral * 0.75 + error;
  integral = constrain(integral, -5000, 5000);
  
  float output = (KP * error) + (KI * integral) + (KD * errorDerivative);
  
  if (PREDICTIVE_TURNING) {
    output += secondDerivative * PREDICTIVE_TURN_FACTOR;
  }
  
  lastError = error;
  return output;
}

// Ultra-sharp turn handler with edge sensor override.
// Sensor on digital pin 2 (index 0) is the rightmost sensor; sensor on pin 9 (index 7) is the leftmost.
void handleUltraSharpTurn(int sensorByte, int &leftSpeed, int &rightSpeed) {
  if (bitRead(sensorByte, 0)) {
    leftSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
    rightSpeed = -SHARP_TURN_SPEED / 2;
    inSharpTurn = true;
    sharpTurnStartTime = millis();
    lastDirection = 2;
    directionConfidence = DIRECTION_HYSTERESIS + 2;
    return;
  }
  
  if (bitRead(sensorByte, NUM_SENSORS - 1)) {
    leftSpeed = -SHARP_TURN_SPEED / 2;
    rightSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
    inSharpTurn = true;
    sharpTurnStartTime = millis();
    lastDirection = 1;
    directionConfidence = DIRECTION_HYSTERESIS + 2;
    return;
  }
  
  bool extremeLeftTurn = (sensorByte & B11000000) != 0 && (sensorByte & B00111111) == 0;
  bool extremeRightTurn = (sensorByte & B00000011) != 0 && (sensorByte & B11111100) == 0;
  
  if (extremeLeftTurn) {
    if ((sensorByte & B10000000) == B10000000) {
      leftSpeed = -SHARP_TURN_SPEED/2;
      rightSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
      inSharpTurn = true;
      sharpTurnStartTime = millis();
      lastDirection = 1;
      directionConfidence = DIRECTION_HYSTERESIS + 2;
    } else {
      leftSpeed = EXTREME_TURN_SPEED;
      rightSpeed = NORMAL_SPEED + OUTER_WHEEL_BOOST;
      inSharpTurn = true;
      sharpTurnStartTime = millis();
      lastDirection = 1;
      directionConfidence = DIRECTION_HYSTERESIS + 1;
    }
  } 
  else if (extremeRightTurn) {
    if ((sensorByte & B00000001) == B00000001) {
      leftSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
      rightSpeed = -SHARP_TURN_SPEED/2;
      inSharpTurn = true;
      sharpTurnStartTime = millis();
      lastDirection = 2;
      directionConfidence = DIRECTION_HYSTERESIS + 2;
    } else {
      leftSpeed = NORMAL_SPEED + OUTER_WHEEL_BOOST;
      rightSpeed = EXTREME_TURN_SPEED;
      inSharpTurn = true;
      sharpTurnStartTime = millis();
      lastDirection = 2;
      directionConfidence = DIRECTION_HYSTERESIS + 1;
    }
  }
}

// Enhanced turn handler with pattern recognition and hysteresis
void handleAdvancedTurn(int sensorByte) {
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  recordPattern(sensorByte);
  
  if (sensorByte == NO_LINE) {
    if (millis() - lastLineTime > LINE_TIMEOUT) {
      unsigned long searchStartTime = millis();
      int searchDirection = lastDirection;
      if (searchDirection == 0) {
        float errorSum = 0;
        for (int i = 0; i < 5; i++) {
          errorSum += errorHistory[i];
        }
        searchDirection = (errorSum < 0) ? 1 : 2;
      }
      if (searchDirection == 1) {
        while (millis() - searchStartTime < 300) {
          setMotors(-SHARP_TURN_SPEED, NORMAL_SPEED, "Left Recovery");
          if (readSensorArray() != NO_LINE) return;
          delay(5);
        }
        setMotors(0, 0, "Pausing");
        delay(100);
        setMotors(-SHARP_TURN_SPEED*1.5, NORMAL_SPEED, "Left Wide Recovery");
      } else {
        while (millis() - searchStartTime < 300) {
          setMotors(NORMAL_SPEED, -SHARP_TURN_SPEED, "Right Recovery");
          if (readSensorArray() != NO_LINE) return;
          delay(5);
        }
        setMotors(0, 0, "Pausing");
        delay(100);
        setMotors(NORMAL_SPEED, -SHARP_TURN_SPEED*1.5, "Right Wide Recovery");
      }
      return;
    }
  } else {
    lastLineTime = millis();
  }
  
  if (sensorByte == ALL_LINE) {
    setMotors(NORMAL_SPEED, NORMAL_SPEED, "Intersection");
    return;
  }
  
  handleUltraSharpTurn(sensorByte, leftSpeed, rightSpeed);
  if (leftSpeed != 0 || rightSpeed != 0) {
    setMotors(leftSpeed, rightSpeed, "Ultra Sharp Turn");
    return;
  }
  
  if (inSharpTurn && (millis() - sharpTurnStartTime < MIN_SHARP_TURN_DURATION)) {
    if (lastDirection == 1) {
      setMotors(SHARP_TURN_SPEED, NORMAL_SPEED + OUTER_WHEEL_BOOST, "Maintain Left Turn");
    } else {
      setMotors(NORMAL_SPEED + OUTER_WHEEL_BOOST, SHARP_TURN_SPEED, "Maintain Right Turn");
    }
    return;
  } else {
    inSharpTurn = false;
  }
  
  bool leftBias = false;
  bool rightBias = false;
  int leftCount = 0;
  int rightCount = 0;
  
  for (int i = 0; i < 4; i++) {
    if (bitRead(sensorByte, i)) leftCount++;
  }
  
  for (int i = 4; i < 8; i++) {
    if (bitRead(sensorByte, i)) rightCount++;
  }
  
  leftBias = (leftCount > rightCount + 1);
  rightBias = (rightCount > leftCount + 1);
  
  if (ENABLE_HYSTERESIS) {
    if (leftBias) {
      if (lastDirection == 2) {
        directionConfidence--;
        if (directionConfidence <= 0) {
          lastDirection = 1;
          directionConfidence = DIRECTION_HYSTERESIS;
          lastDirectionChangeTime = millis();
        } else {
          leftBias = false;
        }
      } else {
        lastDirection = 1;
        directionConfidence = DIRECTION_HYSTERESIS;
      }
    }
    
    if (rightBias) {
      if (lastDirection == 1) {
        directionConfidence--;
        if (directionConfidence <= 0) {
          lastDirection = 2;
          directionConfidence = DIRECTION_HYSTERESIS;
          lastDirectionChangeTime = millis();
        } else {
          rightBias = false;
        }
      } else {
        lastDirection = 2;
        directionConfidence = DIRECTION_HYSTERESIS;
      }
    }
  }
  
  float adjustment = calculateAdvancedPID();
  
  if (abs(adjustment) > SHARP_TURN_THRESHOLD) {
    if (adjustment < 0) {
      leftSpeed = SHARP_TURN_SPEED;
      rightSpeed = NORMAL_SPEED + (adjustment < -SHARP_TURN_THRESHOLD*1.5 ? OUTER_WHEEL_BOOST : 0);
      setMotors(leftSpeed, rightSpeed, "Sharp Left from PID");
    } else {
      leftSpeed = NORMAL_SPEED + (adjustment > SHARP_TURN_THRESHOLD*1.5 ? OUTER_WHEEL_BOOST : 0);
      rightSpeed = SHARP_TURN_SPEED;
      setMotors(leftSpeed, rightSpeed, "Sharp Right from PID");
    }
  } else {
    leftSpeed = BASE_SPEED - adjustment;
    rightSpeed = BASE_SPEED + adjustment;
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    setMotors(leftSpeed, rightSpeed, "PID Control");
  }
  
  previousSensorPattern = sensorByte;
}

void configureResponseProfile(int mode) {
  switch(mode) {
    case 1:
      KP = 0.65;
      KI = 0.01;
      KD = 0.3;
      break;
    case 2:
      KP = 0.85;
      KI = 0.02;
      KD = 0.5;
      break;
    case 3:
      KP = 1.1;
      KI = 0.02;
      KD = 0.7;
      break;
  }
}

void setup() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  Serial.begin(115200);
  Serial.println("High Performance Line Follower Initializing...");
  
  // Initial calibration using outer sensors.
  calibrateSensors();
  
  configureResponseProfile(RESPONSE_MODE);
  
  delay(1000);
}

void loop() {
  unsigned long cycleStartTime = micros();
  
  // Read the sensor array (which updates sensorValues)
  int sensorByte = readSensorArray();
  // Compute the current line position.
  float pos = calculatePosition();
  // Only update dynamic line type if the robot is roughly centered (on a straight line).
  if (abs(pos) < 10) { // threshold value can be adjusted
    dynamicUpdateLineType();
  }
  
  handleAdvancedTurn(sensorByte);
  
  unsigned long cycleTime = micros() - cycleStartTime;
  cycleTimeTotal += cycleTime;
  cycleCount++;
  
  if (cycleCount >= 100) {
    Serial.print("Avg cycle time: ");
    Serial.print(cycleTimeTotal / cycleCount);
    Serial.println(" μs");
    cycleTimeTotal = 0;
    cycleCount = 0;
  }
  
  unsigned long processingTime = micros() - cycleStartTime;
  unsigned long targetCycleTime = 1000000 / PROCESSING_FREQUENCY;
  
  if (processingTime < targetCycleTime) {
    delayMicroseconds(targetCycleTime - processingTime);
  }
}
