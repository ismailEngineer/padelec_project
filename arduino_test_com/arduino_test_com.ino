#include <SoftwareSerial.h>

// RX, TX côté Arduino
SoftwareSerial stm32Serial(8, 9);  // RX=D8, TX=D9

void setup() {
  Serial.begin(115200);       // USB <-> PC
  stm32Serial.begin(57600);   // UART <-> STM32

  Serial.println("Arduino ready - USB <-> STM32 bridge");
}

void loop() {
  // ---------- PC -> STM32 ----------
  if (Serial.available()) {
    char c = Serial.read();
    stm32Serial.write(c);
  }

  // ---------- STM32 -> Arduino ----------
  if (stm32Serial.available()) {
    char c = stm32Serial.read();

    // Affichage sur le Serial Monitor
    Serial.print("STM32 -> ");
    Serial.println(c);

    // (optionnel) renvoyer au PC
    Serial.write(c);
  }
}
