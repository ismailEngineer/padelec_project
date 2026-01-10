#include <SoftwareSerial.h>

#define STM32_RX 8   // Arduino reçoit depuis STM32 TX
#define STM32_TX 9   // Arduino envoie vers STM32 RX

SoftwareSerial stm32Serial(STM32_RX, STM32_TX);

void setup() {
  Serial.begin(9600);        // USB (Python)
  stm32Serial.begin(9600);  // UART vers STM32

  Serial.println("Bridge USB <-> STM32 prêt");
}

void loop() {
  // Python -> STM32
  while (Serial.available()) {
    char c = Serial.read();
    stm32Serial.write(c);
  }

  // STM32 -> Python (optionnel)
  while (stm32Serial.available()) {
    char c = stm32Serial.read();
    Serial.write(c);
  }
}
