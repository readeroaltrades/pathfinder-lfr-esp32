/*
  Pathfinder LFR ESP32 - Basic Line Following Boilerplate
  --------------------------------------------------------
  - Reads 3 TCRT5000 sensors (L, M, R)
  - Controls 2 DC motors via L298N
  - Basic line following logic
  - Prints sensor values for calibration
*/

#define LEFT_SENSOR   34
#define MID_SENSOR    35
#define RIGHT_SENSOR  32

#define ENA 25  // Left motor speed control (PWM)
#define IN1 26
#define IN2 27
#define ENB 33  // Right motor speed control (PWM)
#define IN3 14
#define IN4 12

// Motor speed (0–255 for analogWrite)
int baseSpeed = 180;
int turnSpeed = 150;

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

  Serial.println("Pathfinder LFR Booting...");
  delay(1000);
}

void loop() {
  int leftVal = analogRead(LEFT_SENSOR);
  int midVal = analogRead(MID_SENSOR);
  int rightVal = analogRead(RIGHT_SENSOR);

  // Print raw values for calibration
  Serial.print("L: "); Serial.print(leftVal);
  Serial.print(" M: "); Serial.print(midVal);
  Serial.print(" R: "); Serial.println(rightVal);

  // Determine surface: dark = low value, white = high value
  int threshold = 2000; // Tune this based on track and sensor

  bool leftOnLine = (leftVal < threshold);
  bool midOnLine = (midVal < threshold);
  bool rightOnLine = (rightVal < threshold);

  if (midOnLine && !leftOnLine && !rightOnLine) {
    // Go straight
    moveForward(baseSpeed);
  }
  else if (leftOnLine && !midOnLine) {
    // Turn left
    turnLeft(turnSpeed);
  }
  else if (rightOnLine && !midOnLine) {
    // Turn right
    turnRight(turnSpeed);
  }
  else if (midOnLine && leftOnLine && rightOnLine) {
    // Intersection or start/finish
    moveForward(baseSpeed);
  }
  else {
    // Line lost — stop or slow down
    stopMotors();
  }

  delay(20);
}

// Motor control functions
void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed / 2);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed / 2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
