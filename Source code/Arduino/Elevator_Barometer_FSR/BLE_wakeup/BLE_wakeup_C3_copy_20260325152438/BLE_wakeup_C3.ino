#include <NimBLEDevice.h>

#define SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID_RX  "87654321-4321-4321-4321-cba987654321"
#define CHAR_UUID_TX  "11111111-1111-1111-1111-111111111111"

NimBLECharacteristic* pTxChar = nullptr;
volatile bool isAwake = false;

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
        sendMessage("ESP32-C3 connected and ready");
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("[BLE] Client disconnected, restarting advertising...");
        NimBLEDevice::getAdvertising()->start();
    }
};

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== ESP32-C3 BLE Message Demo ===");

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
        // Simulate going to sleep with a delay
        delay(5000);  // Adjust this as needed
    } else {
        isAwake = false;
        sendMessage("Going to sleep...");
        Serial.println("[SLEEP] Going back to sleep");
        delay(200);
        // Simulate going to sleep with a delay
        delay(5000);  // Adjust this as needed
    }
}