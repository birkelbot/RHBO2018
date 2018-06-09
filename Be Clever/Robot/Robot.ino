
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>

#define NUM_INSTRUCTIONS 3
#define MAX_MS_NO_COMMS 250

#define LED_BUILTIN 13
#define ARM_DOWN_POS 19
#define ARM_MED_POS 100
#define ARM_UP_POS 180
#define CLAW_OPEN_POS 50
#define CLAW_MED_POS 70
#define CLAW_CLOSED_POS 83
#define BED_DOWN_POS 175
#define BED_DUMP_POS 80

// Motors
MeDCMotor left_motor(M1);
MeDCMotor right_motor(M2);

// Servo
MePort servo_port_arm(PORT_4);
MePort servo_port_claw(PORT_2);
MePort servo_port_bed(PORT_1);
Servo arm_servo;
Servo claw_servo;
Servo bed_servo;

// Fun things!`
MeBuzzer buzzer;

bool bed_attached = false;

void setup() {
  // Setup.
  Serial.begin(115200);    // The factory default baud rate is 115200
  arm_servo.attach(servo_port_arm.pin1());
  arm_servo.write(ARM_DOWN_POS);

  claw_servo.attach(servo_port_claw.pin1());
  claw_servo.write(CLAW_OPEN_POS);

  // Give a few blinks to show that the code is up and running.
  blink(3);
}

unsigned long last_time_rx = 0;

void loop() {
  tryReadComms();
  checkComms();
}

void tryReadComms() {
  // Wait until enough instructions have arrived.
  if (Serial.available() < NUM_INSTRUCTIONS + 1) {
    return;
  }

  // Look for the start byte (255, or 0xFF)
  int start = Serial.read();
  if (start != 255) {
    return;
  }

  // Indicate that we have signal by illuminating the on-board LED
  digitalWrite(LED_BUILTIN, HIGH);
  last_time_rx = millis();

  // Read the instructions.
  int left_motor_speed = Serial.read();
  int right_motor_speed = Serial.read();
  int bitwiseCmd = Serial.read();

  int arm_raw = bitwiseCmd & 3;
  int claw_raw = (bitwiseCmd >> 2) & 3;
  int bed_raw = (bitwiseCmd >> 4) & 1;
  int horn = (bitwiseCmd >> 5) & 1;

  int arm_pos = 0, claw_pos = 0, bed_pos = 0;

  switch (arm_raw) {
    case 0:
      arm_pos = ARM_DOWN_POS;
      break;
    case 1:
      arm_pos = ARM_MED_POS;
      break;
    case 2:
      arm_pos = ARM_UP_POS;
      break;
  }

  switch (claw_raw) {
    case 0:
      claw_pos = CLAW_CLOSED_POS;
      break;
    case 1:
      claw_pos = CLAW_MED_POS;
      break;
    case 2:
      claw_pos = CLAW_OPEN_POS;
      break;
  }

  setMotorSpeed(left_motor_speed, &left_motor);
  setMotorSpeed(right_motor_speed, &right_motor);
  arm_servo.write(arm_pos);
  claw_servo.write(claw_pos);

  if (horn) {
    buzzer.tone(/*hz*/ 400, /*ms*/ 300);
  } else if (left_motor_speed < 126 && right_motor_speed > 129) {
    buzzer.tone(/*hz*/ 600, /*ms*/ 100);
  } else {
    buzzer.noTone();
  }

  if (bed_raw == 1) {
    attachBed();
  } else {
    detachBed();
  }
}

void attachBed() {
  if (bed_attached) return;
  bed_servo.attach(servo_port_bed.pin1());
  bed_servo.write(BED_DUMP_POS);
  bed_attached = true;
}

void detachBed() {
  if (!bed_attached) return;
  bed_servo.write(BED_DOWN_POS);
  delay(200);
  bed_servo.detach();
  bed_attached = false;
}


void checkComms() {
  if (millis() - last_time_rx > MAX_MS_NO_COMMS) {
    // Set all motors to neutral
    left_motor.run(0);
    right_motor.run(0);
    arm_servo.write(ARM_DOWN_POS);
    claw_servo.write(CLAW_OPEN_POS);
    detachBed();
    // Indicate that we have lost comms by turning off the on-board LED
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("No comms");
  }
}

bool setMotorSpeed(int motor_speed, MeDCMotor* motor) {
  // Motor speed is betwen 0-254, map it back to -255, 255.
  if (motor_speed >= 0 && motor_speed <= 254) {
    motor_speed *= 2;
    motor_speed -= 254;
    motor->run(motor_speed);
  }
}

void blink(int num_blinks) {
  for (int i = 0; i < num_blinks; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
