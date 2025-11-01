/*
  Pathfinder LFR ESP32 - PID Line Follower
  -----------------------------------------
  - Uses 3 analog line sensors (TCRT5000)
  - Controls 2 DC motors via L298N
  - Implements PID control for smooth turns & speed correction
  - Ideal for competition tracks (curves, intersections, sharp turns)
*/

#define LEFT_SENSOR   34
#define MID_SENSOR    35
#define RIGHT_SENSOR  32

#define ENA 25   // Left motor PWM
#define IN1 26
#define IN2 27
#define ENB 33   // Right motor PWM
#define IN3 14
#define IN4 12

// PID constants (tune these!)
float Kp = 25.0;   // Proportional gain
float Ki = 0.0;    // Integral gain (start at 0)
float Kd = 8.0;    // Derivative gain

// Sensor calibration threshold
int threshold = 2000;  // adjust after calibration

// Variables
float error = 0, lastError = 0, integral = 0, derivative = 0;
int baseSpeed = 160;   // base motor speed (0–255)
int maxSpeed  = 255;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(MID_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Pathfinder LFR - PID Control Booting...");
  delay(1500);
}

void loop() {
  // --- Read Sensors ---
  int leftVal  = analogRead(LEFT_SENSOR);
  int midVal   = analogRead(MID_SENSOR);
  int rightVal = analogRead(RIGHT_SENSOR);

  // Convert readings to binary (1 = line, 0 = background)
  bool leftOnLine  = (leftVal < threshold);
  bool midOnLine   = (midVal < threshold);
  bool rightOnLine = (rightVal < threshold);

  // --- Calculate Error ---
  if (leftOnLine && !midOnLine && !rightOnLine)
    error = -2;  // line to the left
  else if (leftOnLine && midOnLine && !rightOnLine)
    error = -1;
  else if (!leftOnLine && midOnLine && !rightOnLine)
    error = 0;   // line centered
  else if (!leftOnLine && midOnLine && rightOnLine)
    error = 1;
  else if (!leftOnLine && !midOnLine && rightOnLine)
    error = 2;   // line to the right
  else if (!leftOnLine && !midOnLine && !rightOnLine)
    error = lastError;  // line lost → continue previous direction

  // --- PID Computation ---
  integral += error;
  derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // --- Motor Speed Adjustments ---
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // --- Apply Speeds ---
  setMotorSpeed(leftSpeed, rightSpeed);

  // --- Debug Info ---
  Serial.print("Err: "); Serial.print(error);
  Serial.print(" | Corr: "); Serial.print(correction);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);

  delay(10);
}

// ================= MOTOR CONTROL =================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Move forward with PWM control
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}