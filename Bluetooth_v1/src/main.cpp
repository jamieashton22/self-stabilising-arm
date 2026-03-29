#include <Arduino.h>

// /dev/tty.HC-05
//ls /dev/tty.*
// screen /dev/tty.HC-05 9600

// > lsof | grep HC-05 to list processes
//> kill -9 4175

#include <SoftwareSerial.h>

void setup() {
  Serial.begin(9600);      // USB monitor (optional, for debugging)
  Serial1.begin(9600);     // Bluetooth
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print("Got: ");  // debug to USB monitor
    Serial.println(c);

    if (c == '1') digitalWrite(13, HIGH);
    if (c == '0') digitalWrite(13, LOW);
  }
}