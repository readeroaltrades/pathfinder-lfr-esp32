/*
  Pathfinder LFR ESP32 - PID Line Follower (Optimized v2)
  --------------------------------------------------------
  - Uses 3 analog line sensors (TCRT5000)
  - Controls 2 DC motors via L298N (with LEDC PWM)
  - Implements PID control with integral windup protection
  - Optimized for N20 1000RPM motors with better speed management
  
  IMPROVEMENTS:
  - Added integral windup protection
  - Lowered PWM frequency for better motor response
  - More conservative speed limits for better control
  - Added startup calibration helper
  - Better sensor threshold handling
*/

#include <Arduino.h>

// ---------------- Pin Definitions ----------------
#define LEFT_SENSOR   34
#define MID_SENSOR    35
#define RIGHT_SENSOR  32

#define ENA 33    // Left motor PWM
#define IN1 25    // Left motor direction
#define IN2 26
#define ENB 27    // Right motor PWM
#define IN3 14    // Right motor direction
#define IN4 12

// ---------------- PID Constants ----------------
float Kp = 25.0;   // Proportional gain (tune this first)
float Ki = 0.0;    // Integral gain (keep low or 0 initially)
float Kd = 8.0;    // Derivative gain (adds stability)

// ---------------- Sensor Calibration ----------------
int threshold = 1600;  // Black/White threshold (adjust after calibration)
int blackValue = 0;    // Sensor reading on black line
int whiteValue = 3200; // Sensor reading on white surface

// ---------------- PID Variables ----------------
float error = 0, lastError = 0, integral = 0, derivative = 0;
const float integralLimit = 100.0;  // Prevent integral windup

// ---------------- Motor Speed Settings ----------------
int baseSpeed = 180;   // Base speed (start conservative)
int maxSpeed  = 220;   // Max speed limit (leaves room for PID corrections)
int minSpeed  = 50;    // Minimum speed to prevent motor stall

// ---------------- PWM (LEDC) Setup ----------------
#define PWM_FREQ 5000    // 5 kHz - better for N20 motors with L298N
#define PWM_RES  8       // 8-bit resolution (0–255)
#define PWM_LEFT  0
#define PWM_RIGHT 1

// ---------------- Setup Functions ----------------
void setupMotors() {
  // Configure PWM channels
  ledcSetup(PWM_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_RIGHT, PWM_FREQ, PWM_RES);
  
  // Attach PWM to motor enable pins
  ledcAttachPin(ENA, PWM_LEFT);
  ledcAttachPin(ENB, PWM_RIGHT);
  
  // Set motor direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Default direction: forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ---------------- Motor Control Function ----------------
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Clamp speed values
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply PWM duty cycle
  ledcWrite(PWM_LEFT, leftSpeed);
  ledcWrite(PWM_RIGHT, rightSpeed);
}

// ---------------- Stop Motors ----------------
void stopMotors() {
  ledcWrite(PWM_LEFT, 0);
  ledcWrite(PWM_RIGHT, 0);
}

// ---------------- Sensor Calibration Helper ----------------
void calibrateSensors() {
  Serial.println("\n=== SENSOR CALIBRATION ===");
  Serial.println("Place sensors over WHITE surface...");
  delay(3000);
  
  int whiteL = 0, whiteM = 0, whiteR = 0;
  for(int i = 0; i < 50; i++) {
    whiteL += analogRead(LEFT_SENSOR);
    whiteM += analogRead(MID_SENSOR);
    whiteR += analogRead(RIGHT_SENSOR);
    delay(20);
  }
  whiteL /= 50; whiteM /= 50; whiteR /= 50;
  
  Serial.println("Place sensors over BLACK line...");
  delay(3000);
  
  int blackL = 0, blackM = 0, blackR = 0;
  for(int i = 0; i < 50; i++) {
    blackL += analogRead(LEFT_SENSOR);
    blackM += analogRead(MID_SENSOR);
    blackR += analogRead(RIGHT_SENSOR);
    delay(20);
  }
  blackL /= 50; blackM /= 50; blackR /= 50;
  
  Serial.println("\n--- Results ---");
  Serial.printf("White: L=%d M=%d R=%d\n", whiteL, whiteM, whiteR);
  Serial.printf("Black: L=%d M=%d R=%d\n", blackL, blackM, blackR);
  
  int avgWhite = (whiteL + whiteM + whiteR) / 3;
  int avgBlack = (blackL + blackM + blackR) / 3;
  threshold = (avgWhite + avgBlack) / 2;
  
  Serial.printf("\nRecommended threshold: %d\n", threshold);
  Serial.println("Update threshold in code if needed.\n");
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(MID_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  setupMotors();
  stopMotors();

  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║  Pathfinder LFR - Optimized v2    ║");
  Serial.println("╚════════════════════════════════════╝");
  
  // Uncomment to run calibration
  // calibrateSensors();
  
  Serial.println("\nStarting in 3 seconds...");
  Serial.printf("Base Speed: %d | Max Speed: %d\n", baseSpeed, maxSpeed);
  Serial.printf("PID: Kp=%.1f Ki=%.1f Kd=%.1f\n\n", Kp, Ki, Kd);
  delay(3000);
}

// ---------------- Main Loop ----------------
void loop() {
  // --- Read Sensors ---
  int leftVal  = analogRead(LEFT_SENSOR);
  int midVal   = analogRead(MID_SENSOR);
  int rightVal = analogRead(RIGHT_SENSOR);
  
  // --- Determine Line Position ---
  bool leftOnLine  = (leftVal < threshold);
  bool midOnLine   = (midVal < threshold);
  bool rightOnLine = (rightVal < threshold);
  
  // --- Calculate Error (Position Error) ---
  if (leftOnLine && !midOnLine && !rightOnLine) {
    error = -2;  // Line far left - turn left hard
  }
  else if (leftOnLine && midOnLine && !rightOnLine) {
    error = -1;  // Line slightly left
  }
  else if (!leftOnLine && midOnLine && !rightOnLine) {
    error = 0;   // Line centered - perfect!
  }
  else if (!leftOnLine && midOnLine && rightOnLine) {
    error = 1;   // Line slightly right
  }
  else if (!leftOnLine && !midOnLine && rightOnLine) {
    error = 2;   // Line far right - turn right hard
  }
  else if (leftOnLine && midOnLine && rightOnLine) {
    error = 0;   // Wide line or intersection - go straight
  }
  else if (!leftOnLine && !midOnLine && !rightOnLine) {
    // Line lost - maintain last correction
    error = lastError;
  }

  // --- PID Computation ---
  integral += error;
  integral = constrain(integral, -integralLimit, integralLimit);  // Anti-windup
  
  derivative = error - lastError;
  
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  lastError = error;

  // --- Compute Motor Speeds ---
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Apply speed limits
  leftSpeed  = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  // --- Apply Speeds ---
  setMotorSpeed(leftSpeed, rightSpeed);

  // --- Debug Info (reduce frequency if needed) ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {  // Print every 100ms
    Serial.print("Sensors[L:"); Serial.print(leftVal);
    Serial.print(" M:"); Serial.print(midVal);
    Serial.print(" R:"); Serial.print(rightVal);
    Serial.print("] | Err:"); Serial.print(error);
    Serial.print(" | Corr:"); Serial.print(correction, 1);
    Serial.print(" | Speed[L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.print(rightSpeed);
    Serial.println("]");
    lastPrint = millis();
  }
  
  delay(10);  // 100Hz control loop
}