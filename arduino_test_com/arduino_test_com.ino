#include <SoftwareSerial.h>

SoftwareSerial stmSerial(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);       // PC
  stmSerial.begin(115200);    // STM32
}

void loop() {
  // PC → STM32
  if (Serial.available()) {
    stmSerial.write(Serial.read());
  }

  // STM32 → PC
  if (stmSerial.available()) {
    Serial.write(stmSerial.read());
  }
}
