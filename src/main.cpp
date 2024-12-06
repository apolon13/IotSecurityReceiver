#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "IoTRadioSignal.h"
#include "Preferences.h"

Preferences preferences;

typedef struct {
    bool activateScan;
    bool alarm;
    bool ping;
} Message;

typedef struct {
    String value;
} Sensor;

typedef struct {
    bool alarm;
    bool ping;
} SirenMessage;

TaskHandle_t handler;
IoTRadioSignal *ioTRadioSignal;
Message msg;

uint8_t sirenMacAddress[] = {0xA0, 0xA3, 0xB3, 0x2A, 0xF9, 0x6C};
uint8_t stationMacAddress[] = {0xEC, 0xDA, 0x3B, 0x97, 0x2A, 0xA8};

void scanSensorsTask(void *) {
    while (true) {
        string value;
        ioTRadioSignal->scan(value);
        if (!value.empty()) {
            Sensor sensor = {value.c_str()};
            esp_now_send(stationMacAddress, (uint8_t *) &sensor, sizeof(sensor));
        }
        value = "";
        vTaskDelay(10);
    }
}

void handleScanTask(bool activateScan) {
    if (activateScan) {
        ioTRadioSignal->enable();
        if (handler == nullptr) {
            xTaskCreate(
                    scanSensorsTask,
                    "scanSensorsTask",
                    5000,
                    NULL,
                    1,
                    &handler
            );
        } else {
            vTaskResume(handler);
        }
    }

    if (!activateScan && handler != nullptr) {
        ioTRadioSignal->disable();
        vTaskSuspend(handler);
    }

    preferences.putBool("scanIsActive", activateScan);
}

void handleAlarm(bool alarm) {
    auto currentAlarmStatus = preferences.getBool("alarmIsActive", alarm);
    if (currentAlarmStatus != alarm) {
        SirenMessage sirenMessage = {.alarm = currentAlarmStatus, .ping = false};
        esp_now_send(sirenMacAddress, (uint8_t *) &sirenMessage, sizeof(sirenMessage));
        preferences.putBool("alarmIsActive", alarm);
    }
}

void handlePing(bool ping) {
    auto currentAlarmStatus = preferences.getBool("alarmIsActive", alarm);
    SirenMessage sirenMessage = {.alarm =  currentAlarmStatus, .ping =  ping};
    esp_now_send(sirenMacAddress, (uint8_t *) &sirenMessage, sizeof(sirenMessage));
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&msg, incomingData, sizeof(msg));
    handleScanTask(msg.activateScan);
    handleAlarm(msg.alarm);
    handlePing(msg.ping);
}

esp_now_peer_info_t StationPeer;
esp_now_peer_info_t SirenPeer;

void setup() {
    Serial.begin(9600);
    ioTRadioSignal = new IoTRadioSignal(GPIO_NUM_15);
    preferences.begin("esp-receiver", false);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        return;
    }
    memcpy(StationPeer.peer_addr, stationMacAddress, 6);
    StationPeer.encrypt = false;
    if (esp_now_add_peer(&StationPeer) != ESP_OK) {
        return;
    }

    memcpy(SirenPeer.peer_addr, sirenMacAddress, 6);
    SirenPeer.encrypt = false;
    if (esp_now_add_peer(&SirenPeer) != ESP_OK) {
        return;
    }

    WiFi.printDiag(Serial);
    esp_now_register_recv_cb(OnDataRecv);

    handleScanTask(preferences.getBool("scanIsActive", true));
    handleAlarm(preferences.getBool("alarmIsActive", false));
}

void loop() {
}