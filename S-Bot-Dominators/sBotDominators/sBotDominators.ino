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
const int SLIGHT_TURN_SPEED = 120;   // Speed for slight turns (reduced)
const int SHARP_TURN_SPEED = 80;     // Speed for sharp turns (reduced)
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

void setup() {
  // Set appropriate pin modes
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Set initial motor state - ensure robot is stopped
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Begin serial communication for debugging
  Serial.begin(115200);  // Increased baud rate for better debugging
  Serial.println("High Performance Line Follower Initialized");
  
  // Configure PID based on selected response mode
  configureResponseProfile(RESPONSE_MODE);
  
  // Brief pause before starting
  delay(1000);
}

// Configure robot behavior based on selected response profile
void configureResponseProfile(int mode) {
  switch(mode) {
    case 1:  // Smooth following - good for gradual curves
      KP = 0.65;
      KI = 0.01;
      KD = 0.3;
      break;
    case 2:  // Responsive - balanced for most tracks
      KP = 0.85;
      KI = 0.02;
      KD = 0.5;
      break;
    case 3:  // Aggressive - for tracks with sharp turns
      KP = 1.1;
      KI = 0.02;
      KD = 0.7;
      break;
  }
}

int readSensorsQuick() {
  int sensorByte = 0;
  
  // Fast digital reading - prioritizes speed
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  
  return sensorByte;
}

int readSensorsMultiSample() {
  int sensorByte = 0;
  
  // Take 3 samples and use majority vote
  for (int i = 0; i < NUM_SENSORS; i++) {
    int votes = digitalRead(sensorPins[i]);
    delayMicroseconds(100);
    votes += digitalRead(sensorPins[i]);
    delayMicroseconds(100);
    votes += digitalRead(sensorPins[i]);
    
    sensorValues[i] = (votes >= 2) ? 1 : 0;
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  
  return sensorByte;
}

int readSensorsAnalog() {
  int sensorByte = 0;
  
  // Analog reading with threshold
  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(sensorPins[i]);
    sensorValues[i] = (reading < 500) ? 1 : 0;  // Adjust threshold as needed
    bitWrite(sensorByte, i, sensorValues[i]);
  }
  
  return sensorByte;
}

int readSensorArray() {
  // Choose reading method based on configuration
  switch(SENSOR_READ_MODE) {
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
    if (patternValues[i] == pattern && 
        (currentTime - lastPatterns[i]) < PATTERN_MEMORY_DURATION) {
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
  
  // Log state if debugging is enabled
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(" L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
}

// Emergency stop - rapidly halts the robot
void emergencyBrake() {
  // Cut power but maintain active braking
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(50);  // Brief brake period
  
  // Then release to idle
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Advanced PID controller with turn prediction
float calculateAdvancedPID() {
  float position = calculatePosition();
  float error = position;
  
  // Update error history for derivatives
  errorHistory[historyIndex] = error;
  historyIndex = (historyIndex + 1) % 5;
  
  // Calculate primary derivative (rate of change)
  errorDerivative = error - lastError;
  
  // Calculate secondary derivative (acceleration of error)
  int prevIndex = (historyIndex + 4) % 5;
  int prevPrevIndex = (historyIndex + 3) % 5;
  secondDerivative = (errorHistory[prevIndex] - errorHistory[prevPrevIndex]) - errorDerivative;
  
  // Update integral with anti-windup
  integral = integral * 0.75 + error;
  integral = constrain(integral, -5000, 5000);
  
  // Calculate base PID output
  float output = (KP * error) + (KI * integral) + (KD * errorDerivative);
  
  // Add predictive component if enabled
  if (PREDICTIVE_TURNING) {
    output += secondDerivative * PREDICTIVE_TURN_FACTOR;
  }
  
  lastError = error;
  return output;
}

// Updated Ultra-sharp turn handler with edge sensor override.
// Now sensor on digital pin 2 (index 0) is the rightmost sensor and sensor on digital pin 9 (index 7) is the leftmost.
void handleUltraSharpTurn(int sensorByte, int &leftSpeed, int &rightSpeed) {
  // Check rightmost sensor (sensor index 0, digital pin 2)
  if (bitRead(sensorByte, 0)) {
    // Right edge sensor triggered: reverse right motor
    leftSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
    rightSpeed = -SHARP_TURN_SPEED / 2;
    inSharpTurn = true;
    sharpTurnStartTime = millis();
    lastDirection = 2;  // right turn
    directionConfidence = DIRECTION_HYSTERESIS + 2;
    return;
  }
  
  // Check leftmost sensor (sensor index NUM_SENSORS-1, digital pin 9)
  if (bitRead(sensorByte, NUM_SENSORS - 1)) {
    // Left edge sensor triggered: reverse left motor
    leftSpeed = -SHARP_TURN_SPEED / 2;
    rightSpeed = NORMAL_SPEED + EXTREME_OUTER_BOOST;
    inSharpTurn = true;
    sharpTurnStartTime = millis();
    lastDirection = 1;  // left turn
    directionConfidence = DIRECTION_HYSTERESIS + 2;
    return;
  }
  
  // Otherwise, use original extreme turn detection logic:
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
  
  // Record this pattern for history tracking
  recordPattern(sensorByte);
  
  // Check line timeout first
  if (sensorByte == NO_LINE) {
    if (millis() - lastLineTime > LINE_TIMEOUT) {
      // Invoke intelligent line recovery if no line is detected
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
  
  // Handle intersection detection
  if (sensorByte == ALL_LINE) {
    setMotors(NORMAL_SPEED, NORMAL_SPEED, "Intersection");
    return;
  }
  
  // Check for ultra-sharp turns (this now handles edge sensors)
  handleUltraSharpTurn(sensorByte, leftSpeed, rightSpeed);
  if (leftSpeed != 0 || rightSpeed != 0) {
    setMotors(leftSpeed, rightSpeed, "Ultra Sharp Turn");
    return;
  }
  
  // If we're in a sharp turn, maintain it for the minimum duration
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
  
  // Calculate sensor distribution for progressive turn patterns
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
  
  // Apply hysteresis to avoid oscillation
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
  
  // Use PID control if no ultra-sharp turn is detected
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

void loop() {
  unsigned long cycleStartTime = micros();
  
  int sensorByte = readSensorArray();
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
