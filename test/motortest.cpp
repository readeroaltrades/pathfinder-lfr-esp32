#include <Arduino.h>

// Motor pins
#define ENA 33    // Left motor PWM
#define IN1 25    // Left motor direction pin 1
#define IN2 26    // Left motor direction pin 2
#define ENB 27    // Right motor PWM
#define IN3 14    // Right motor direction pin 1
#define IN4 12    // Right motor direction pin 2

void setup() {
  Serial.begin(115200);
  
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.println("Motor Test Starting...");
  Serial.println("LIFT WHEELS OFF GROUND!");
  delay(3000);  // 3 second delay to lift robot
}

void loop() {
  // Test 1: Both motors forward at medium speed
  Serial.println("Test 1: Both Forward (Speed 150)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(2000);
  
  // Test 2: Left motor only
  Serial.println("Test 2: Left Motor Only");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 0);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENA, 0);
  delay(2000);
  
  // Test 3: Right motor only
  Serial.println("Test 3: Right Motor Only");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, 150);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENB, 0);
  delay(2000);
  
  // Test 4: Both motors backward
  Serial.println("Test 4: Both Backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(5000);
  
  Serial.println("Test Complete! Restarting in 5 seconds...\n");
}