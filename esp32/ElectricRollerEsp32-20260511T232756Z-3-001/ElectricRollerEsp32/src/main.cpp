#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

#define DEVICE_NAME "ElectricSkate-ESP32"

#define SERVICE_UUID        "00000000-0000-0000-0000-000000000000"
#define COMMAND_CHAR_UUID   "00000000-0000-0000-0000-000000000001"
#define TELEMETRY_CHAR_UUID "00000000-0000-0000-0000-000000000002"

BLEServer* server = nullptr;
BLECharacteristic* commandCharacteristic = nullptr;
BLECharacteristic* telemetryCharacteristic = nullptr;

bool deviceConnected = false;
bool previousDeviceConnected = false;

float speed = 12.3;
float totalMileage = 42.300;
float lastTripDistance = 5.100;
int batteryLevel = 87;
bool locked = false;
std::string drivingMode = "NORMAL";

unsigned long lastTelemetryMillis = 0;

void setCommandResponse(const std::string& response) {
  if (commandCharacteristic == nullptr) {
    return;
  }

  commandCharacteristic->setValue((uint8_t*)response.c_str(), response.length());

  Serial.print("Command response <= ");
  Serial.println(response.c_str());
}

void notifyText(const std::string& text) {
  if (!deviceConnected || telemetryCharacteristic == nullptr) {
    return;
  }

  telemetryCharacteristic->setValue((uint8_t*)text.c_str(), text.length());
  telemetryCharacteristic->notify();

  Serial.print("Notify => ");
  Serial.println(text.c_str());
}

std::string buildTelemetryJson() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"speed\":%.1f,\"totalMileage\":%.3f,\"lastTripDistance\":%.3f,"
    "\"batteryLevel\":%d,\"locked\":%s,\"drivingMode\":\"%s\"}",
    speed, totalMileage, lastTripDistance, batteryLevel,
    locked ? "true" : "false", drivingMode.c_str());
  return std::string(buf);
}

void notifyTelemetryNow() {
  notifyText(buildTelemetryJson());
}

void handleCommand(std::string command) {
  Serial.print("Command received => ");
  Serial.println(command.c_str());

  if (command == "HELLO") {
    setCommandResponse("HELLO_FROM_ESP32");
    return;
  }

  if (command == "LOCK") {
    locked = true;
    speed = 0.0;

    setCommandResponse("{\"locked\":true}");
    notifyTelemetryNow();
    return;
  }

  if (command == "UNLOCK") {
    locked = false;
    speed = 12.3;

    setCommandResponse("{\"locked\":false}");
    notifyTelemetryNow();
    return;
  }

  if (command == "SET_DRIVE_MODE:ECO") {
    drivingMode = "ECO";
    speed = locked ? 0.0 : 8.5;

    setCommandResponse("{\"drivingMode\":\"ECO\"}");
    notifyTelemetryNow();
    return;
  }

  if (command == "SET_DRIVE_MODE:NORMAL") {
    drivingMode = "NORMAL";
    speed = locked ? 0.0 : 12.3;

    setCommandResponse("{\"drivingMode\":\"NORMAL\"}");
    notifyTelemetryNow();
    return;
  }

  if (command == "SET_DRIVE_MODE:SPORT") {
    drivingMode = "SPORT";
    speed = locked ? 0.0 : 18.7;

    setCommandResponse("{\"drivingMode\":\"SPORT\"}");
    notifyTelemetryNow();
    return;
  }

  std::string error = "{\"error\":\"UNKNOWN_COMMAND\",\"command\":\"";
  error += command;
  error += "\"}";

  setCommandResponse(error);
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("Flutter app connected");
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("Flutter app disconnected");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string value = characteristic->getValue();

    if (value.length() > 0) {
      handleCommand(value);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting Electric Skate BLE firmware...");

  BLEDevice::init(DEVICE_NAME);

  BLEDevice::setMTU(185);

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  commandCharacteristic = service->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );

  commandCharacteristic->setCallbacks(new CommandCallbacks());
  commandCharacteristic->setValue("READY");

  telemetryCharacteristic = service->createCharacteristic(
    TELEMETRY_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  telemetryCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("BLE server ready");
  Serial.println("Waiting for Flutter app...");
}

void loop() {
  unsigned long now = millis();

  if (deviceConnected && now - lastTelemetryMillis >= 500) {
    notifyTelemetryNow();
    lastTelemetryMillis = now;
  }

  if (!deviceConnected && previousDeviceConnected) {
    delay(500);
    server->startAdvertising();
    Serial.println("Restarted BLE advertising");
    previousDeviceConnected = false;
  }

  if (deviceConnected && !previousDeviceConnected) {
    previousDeviceConnected = true;
  }
}
