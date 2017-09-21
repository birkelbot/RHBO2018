
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeMCore.h>

#define NUM_INSTRUCTIONS 3

#define LED_BUILTIN 13

// Motors
MeDCMotor left_motor(M1);
MeDCMotor right_motor(M2);

// Servo
MePort servo_port(PORT_1);
Servo arm_servo;

void setup() {
  // Setup.
  Serial.begin(115200);    //The factory default baud rate is 115200
  arm_servo.attach(servo_port.pin1());

  // Give a few blinks to show that the code is up and running.
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void loop() {
  if (Serial.available() < NUM_INSTRUCTIONS + 1) {
    return;
  }
  
  int start = Serial.read();
  // Look for the start byte (255, or 0xFF)
  if (start != 255) {
    return;
  }

  // Read the instructions.
  int left_motor_speed = Serial.read();
  int right_motor_speed = Serial.read();
  int servo = Serial.read();

  if (!checkMotorRange(left_motor_speed) || !checkMotorRange(right_motor_speed)) {
    return;
  } 
  
  left_motor.run(left_motor_speed);
  right_motor.run(right_motor_speed);
}

bool checkMotorRange(int motor_speed) {
  return motor_speed >= -255 && motor_speed <= 255;
}

