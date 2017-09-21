#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeMCore.h>


void setup(){
    Serial.begin(115200);
    Serial.println("Start");
}

void loop(){
    if((Serial.available()) > (0)){
        int val = Serial.read();
        Serial.println(val);

        if (val == 'A') {
          digitalWrite(LED_BUILTIN, HIGH);
        } else {
          digitalWrite(LED_BUILTIN, LOW);
        }
    }else{
        Serial.println("No");
    }
    _delay(1);
    _loop();
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
}

//#include "MeOrion.h"
//#include <SoftwareSerial.h>
//
//MeBluetooth bluetooth(PORT_3);
//
//void setup() {
//  // Setup.
//  bluetooth.begin(115200);    //The factory default baud rate is 115200
//
//  // Give a few blinks to show that the code is up and running.
//  blink(2);
//}
//
//void loop() {
//  if (bluetooth.available() < 2) {
//    blink(2);
//    delay(800);
//    return;
//  }
//  
//  int start = bluetooth.read();
//  // Look for the start byte (255, or 0xFF)
//  if (start !=Serial.read() 'S') {
//    blink(3);
//    delay(800);
//    bluetooth.write('A');
//    return;
//  }
//  // Read the instructions.
//  int led_state = bluetooth.read();
//  blink(4);
//  digitalWrite(LED_BUILTIN, led_state == 'O' ? HIGH : LOW);
//}
//
//void blink(int num_blinks) {
//  for (int i = 0; i < num_blinks; ++i) {
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(500);
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(500);
//  }
//}

