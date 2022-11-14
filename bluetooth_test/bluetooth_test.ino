#include <SoftwareSerial.h>

//SoftwareSerial BTSerial(2, 3);   //bluetooth module Tx:Digital 2 Rx:Digital 3

void setup() {
  Serial.begin(9600);
  Serial.println("hi");  //ATcommand Start
}

void loop() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}