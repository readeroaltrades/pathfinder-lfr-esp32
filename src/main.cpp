/*
  Pathfinder LFR ESP32 - Optimized 100Hz PID Line Follower with Extreme Curve Handling
  -----------------------------------------------------------------------------------
  Hardware:
  - 3x Analog line sensors (TCRT5000) on pins 34, 35, 32
  - 2x DC motors via L298N with LEDC PWM
  - N20 1000RPM motors with 500Hz PWM, 8-bit resolution
  
  Optimizations:
  - True 100Hz control loop (10ms cycle time)
  - Adaptive PID with curve detection
  - Extreme curve / U-turn handling
  - Weighted sensor positioning for smooth curves
  - Sensor smoothing filter to reduce noise
  - Emergency line recovery with directional memory
  - Non-blocking calibration with visual feedback
*/

#include <Arduino.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LEFT_SENSOR   34
#define MID_SENSOR    35
#define RIGHT_SENSOR  32

#define ENA 33
#define INA 25
#define INB 26

#define ENB 27
#define INC 14
#define IND 12

#define BOOT_BTN 0

// ============================================================================
// PID TUNING CONSTANTS
// ============================================================================
float Kp = 45.0;
float Ki = 0.15;
float Kd = 18.0;

const float SHARP_CURVE_KP_MULT = 2.0;
const float SHARP_CURVE_KD_MULT = 0.5;

// ============================================================================
// MOTOR SPEED CONFIGURATION
// ============================================================================
int baseSpeed = 200;
int maxSpeed  = 245;
int minSpeed  = 60;
int sharpTurnSpeed = 140;

// Extreme curve pivot speed
int extremeInnerSpeed = 20; // Very slow inner wheel for pivot

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
int sensorPins[3] = {LEFT_SENSOR, MID_SENSOR, RIGHT_SENSOR};
int sensorMin[3];
int sensorMax[3];

const float SENSOR_ALPHA = 0.8;
float filteredSensors[3] = {0, 0, 0};

// ============================================================================
// PID CONTROL VARIABLES
// ============================================================================
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

const float INTEGRAL_LIMIT = 120.0;
const float INTEGRAL_DECAY = 0.95;

// ============================================================================
// LINE TRACKING STATE
// ============================================================================
unsigned long lineLastSeen = 0;
const unsigned long LINE_LOST_TIMEOUT = 500;
int lastValidDirection = 0;

// ============================================================================
// PWM (LEDC) CONFIGURATION
// ============================================================================
#define PWM_FREQ 500
#define PWM_RES  8
#define PWM_LEFT  0
#define PWM_RIGHT 1

// ============================================================================
// TIMING CONTROL
// ============================================================================
unsigned long loopStartTime = 0;
const unsigned long LOOP_PERIOD = 10;

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================
void setupMotors() {
  ledcSetup(PWM_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_LEFT);
  ledcAttachPin(ENB, PWM_RIGHT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
  digitalWrite(INC, HIGH);
  digitalWrite(IND, LOW);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  ledcWrite(PWM_LEFT, leftSpeed);
  ledcWrite(PWM_RIGHT, rightSpeed);
}

void stopMotors() {
  ledcWrite(PWM_LEFT, 0);
  ledcWrite(PWM_RIGHT, 0);
}

// ============================================================================
// SENSOR CALIBRATION
// ============================================================================
void calibrateSensors(int durationMs = 30000) {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║     SENSOR CALIBRATION MODE         ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.printf("Duration: %d seconds\n", durationMs / 1000);
  Serial.println("→ Move robot slowly over line (black & white)\n");
  
  unsigned long startTime = millis();
  unsigned long lastUpdate = 0;
  int sampleCount = 0;

  for (int i = 0; i < 3; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  while (millis() - startTime < durationMs) {
    for (int i = 0; i < 3; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    sampleCount++;
    if (millis() - lastUpdate >= 1000) {
      int remaining = (durationMs - (millis() - startTime)) / 1000;
      Serial.printf("⏱  Calibrating... %2d seconds remaining | Samples: %d\n", remaining, sampleCount);
      lastUpdate = millis();
    }
    delay(5);
  }

  Serial.println("\n✓ Calibration complete!");
  Serial.println("═══════════════════════════════════════════════");
  for (int i = 0; i < 3; i++) {
    const char* names[] = {"LEFT ", "MID  ", "RIGHT"};
    int range = sensorMax[i] - sensorMin[i];
    Serial.printf("%s: min=%4d  max=%4d  range=%4d ", names[i], sensorMin[i], sensorMax[i], range);
    if (range < 500) Serial.print(" ⚠ LOW CONTRAST!");
    Serial.println();
  }
  Serial.println("═══════════════════════════════════════════════\n");

  stopMotors();
  delay(2000);
}

// ============================================================================
// SENSOR READING & LINE POSITION CALCULATION
// ============================================================================
float readLineError() {
  int rawLeft  = analogRead(LEFT_SENSOR);
  int rawMid   = analogRead(MID_SENSOR);
  int rawRight = analogRead(RIGHT_SENSOR);

  filteredSensors[0] = (SENSOR_ALPHA * rawLeft)  + ((1 - SENSOR_ALPHA) * filteredSensors[0]);
  filteredSensors[1] = (SENSOR_ALPHA * rawMid)   + ((1 - SENSOR_ALPHA) * filteredSensors[1]);
  filteredSensors[2] = (SENSOR_ALPHA * rawRight) + ((1 - SENSOR_ALPHA) * filteredSensors[2]);

  int leftVal  = map(filteredSensors[0], sensorMin[0], sensorMax[0], 0, 1000);
  int midVal   = map(filteredSensors[1], sensorMin[1], sensorMax[1], 0, 1000);
  int rightVal = map(filteredSensors[2], sensorMin[2], sensorMax[2], 0, 1000);

  leftVal  = constrain(leftVal, 0, 1000);
  midVal   = constrain(midVal, 0, 1000);
  rightVal = constrain(rightVal, 0, 1000);

  int totalWeight = leftVal + midVal + rightVal;
  
  float weightedPosition = 0;
  if (totalWeight > 500) {
    weightedPosition = ((leftVal * -1000) + (midVal * 0) + (rightVal * 1000)) / (float)totalWeight;
    weightedPosition *= -1; // Flip sign to match correct left/right
    lineLastSeen = millis();
    if (weightedPosition < -100) lastValidDirection = -1;
    else if (weightedPosition > 100) lastValidDirection = 1;
  } else {
    if (millis() - lineLastSeen > LINE_LOST_TIMEOUT) {
      return lastValidDirection * 4.0;
    }
    return lastError;
  }

  float calculatedError = weightedPosition / 400.0;
  return calculatedError;
}

// ============================================================================
// SHARP & EXTREME CURVE DETECTION
// ============================================================================
bool isSharpCurve(float error) {
  return abs(error) > 1.2;
}

bool isExtremeCurve(float error) {
  return abs(error) > 2.0;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================
float calculatePID(float currentError) {
  bool sharpCurve = isSharpCurve(currentError);
  
  float activeKp = sharpCurve ? (Kp * SHARP_CURVE_KP_MULT) : Kp;
  float activeKd = sharpCurve ? (Kd * SHARP_CURVE_KD_MULT) : Kd;
  
  integral += currentError;
  if ((currentError > 0 && lastError < 0) || (currentError < 0 && lastError > 0)) {
    integral *= INTEGRAL_DECAY;
  }
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  derivative = currentError - lastError;
  
  float correction = (activeKp * currentError) + 
                     (Ki * integral) + 
                     (activeKd * derivative);
  
  lastError = currentError;
  return correction;
}

// ============================================================================
// MAIN SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 3; i++) pinMode(sensorPins[i], INPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  setupMotors();
  stopMotors();

  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║  Pathfinder LFR - Optimized 100Hz      ║");
  Serial.println("║  PWM: 500Hz @ 8-bit (0-255)             ║");
  Serial.println("║  Features: Adaptive PID + Extreme Curves║");
  Serial.println("╚═════════════════════════════════════════╝");

  if (digitalRead(BOOT_BTN) == LOW) {
    Serial.println("⚠  BOOT button detected!");
    Serial.println("→  Quick calibration mode (10 seconds)\n");
    delay(1000);
    calibrateSensors(10000);
  } else {
    Serial.println("Starting auto-calibration in 3 seconds...");
    delay(3000);
    calibrateSensors(30000);
  }

  for (int i = 0; i < 3; i++) filteredSensors[i] = analogRead(sensorPins[i]);

  Serial.println("✓ Robot ready! Starting line following...\n");
  delay(1000);
  loopStartTime = millis();
}

// ============================================================================
// MAIN CONTROL LOOP (100Hz)
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  error = readLineError();
  float correction = calculatePID(error);

  int leftSpeed, rightSpeed;
  float absErr = abs(error);

  if (isExtremeCurve(error)) {
    // Extreme curve / 90° / U-turn pivot
    if (error < 0) {
      leftSpeed  = extremeInnerSpeed;
      rightSpeed = maxSpeed;
    } else {
      leftSpeed  = maxSpeed;
      rightSpeed = extremeInnerSpeed;
    }
  } else if (isSharpCurve(error)) {
    // Sharp curve
    int innerSpeed = baseSpeed - (int)(absErr * 50);
    innerSpeed = constrain(innerSpeed, extremeInnerSpeed, baseSpeed);
    if (error < 0) {
      leftSpeed  = innerSpeed;
      rightSpeed = maxSpeed;
    } else {
      leftSpeed  = maxSpeed;
      rightSpeed = innerSpeed;
    }
  } else {
    // Normal PID curve
    leftSpeed  = baseSpeed - correction;
    rightSpeed = baseSpeed + correction;
    leftSpeed  = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  }

  setMotorSpeed(leftSpeed, rightSpeed);

  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 100) {
    Serial.print("Err: "); Serial.print(error, 2);
    Serial.print(" | PID: "); Serial.print(correction, 1);
    Serial.print(" | L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.print(rightSpeed);
    if (isExtremeCurve(error)) Serial.print(" [EXTREME]");
    else if (isSharpCurve(error)) Serial.print(" [SHARP]");
    Serial.println();
    lastPrint = currentTime;
  }

  unsigned long processingTime = millis() - currentTime;
  if (processingTime < LOOP_PERIOD) delay(LOOP_PERIOD - processingTime);
  loopStartTime = currentTime;
}
