
#include "MeOrion.h"
#include <SoftwareSerial.h>

#define NUM_INSTRUCTIONS 1

#define LED_BUILTIN 13

MeBluetooth bluetooth(PORT_3);

void setup() {
  bluetooth.begin(115200);    //The factory default baud rate is 115200
}

void loop() {
  if (bluetooth.available() < NUM_INSTRUCTIONS + 1) {
    return;
  }
  
  int start = bluetooth.read();
  // Look for the start byte (255, or 0xFF)
  if (start != 255) {
    return;
  }

  // Read the instructions.
  int led_state = bluetooth.read();

  if (led_state) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
