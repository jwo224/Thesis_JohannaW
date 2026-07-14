#include <NimBLEDevice.h>
#include "esp_sleep.h"
#include <ESP32Servo.h>

#define SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID_RX  "87654321-4321-4321-4321-cba987654321"
#define CHAR_UUID_TX  "11111111-1111-1111-1111-111111111111"

#define SERVO_PIN 13  // Change to whatever pin your servo is connected to

NimBLECharacteristic* pTxChar = nullptr;
volatile bool isAwake = false;
Servo myServo;

void sendMessage(const char* msg) {
    if (pTxChar == nullptr) return;
    pTxChar->setValue(msg);
    pTxChar->notify();
    Serial.printf("[BLE TX] %s\n", msg);
    delay(100);
}

class WriteCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        std::string val = pChar->getValue();
        while (!val.empty() && (val.back() == '\n' || val.back() == '\r' || val.back() == ' '))
            val.pop_back();

        Serial.printf("[BLE RX] Received: %s\n", val.c_str());

        if (val == "wake up") {
            isAwake = true;
            sendMessage("Woken up!");
        } else {
            sendMessage("Unknown command");
        }
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.println("[BLE] Client connected");
        delay(500);
        sendMessage("ESP32-C3 connected and ready");
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("[BLE] Client disconnected, restarting advertising...");
        NimBLEDevice::getAdvertising()->start();
    }
};

void moveServo() {
    sendMessage("Moving servo...");
    Serial.println("[SERVO] Moving");

    myServo.write(0);    // Go to 0 degrees
    delay(1000);
    myServo.write(90);   // Go to 90 degrees
    delay(1000);

    sendMessage("Servo done!");
    Serial.println("[SERVO] Done");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== ESP32-C3 BLE + Servo Demo ===");

    myServo.attach(SERVO_PIN);
    myServo.write(90); // Start at center position

    NimBLEDevice::init("ESP32-C3");

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    NimBLECharacteristic* pRxChar = pService->createCharacteristic(
        CHAR_UUID_RX,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new WriteCallback());

    pTxChar = pService->createCharacteristic(
        CHAR_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setMinInterval(160);
    pAdv->setMaxInterval(320);
    pAdv->start();

    Serial.println("Advertising as ESP32-C3");
}

void loop() {
    if (!isAwake) {
        sendMessage("Going to sleep...");
        Serial.println("[SLEEP] Going to sleep");
        delay(200);

        while (!isAwake) {
            delay(100);
        }

    } else {
        moveServo();

        isAwake = false;
        sendMessage("Going to sleep...");
        Serial.println("[SLEEP] Going back to sleep");
        delay(200);

        while (!isAwake) {
            delay(100);
        }
    }
}