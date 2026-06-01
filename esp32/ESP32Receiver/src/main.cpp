#include <WiFi.h>
#include <esp_now.h>

// UART vers STM32
#define RXD2 16
#define TXD2 17

void sendToSTM32(const char* type, int target, int value)
{
  char buffer[64];

  snprintf(buffer, sizeof(buffer),
           "%s;%d;%d\n",
           type,
           target,
           value);

  Serial1.print(buffer);   // UART vers STM32
  Serial.print("TX STM32: ");
  Serial.print(buffer);    // debug USB
}

// ================= ESP-NOW RX =================
void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
  char msg[100];
  memcpy(msg, data, len);
  msg[len] = '\0';

  Serial.print("ESP-NOW RX: ");
  Serial.println(msg);

  // Exemple de parsing simple côté ESP32
  // Format reçu depuis la télécommande :
  // PWM;1;70
  // LED;1;1
  // ADC;0;0

  char *type = strtok(msg, ";");
  char *target = strtok(NULL, ";");
  char *value = strtok(NULL, ";");

  if (!type || !target || !value) return;

  sendToSTM32(type, atoi(target), atoi(value));
}

void setup()
{
  Serial.begin(115200);

  // UART vers STM32
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP32 Bridge Ready");
}

void loop()
{
  delay(10);
}