#include "DFRobot_AHT20.h"
#include <esp_now.h>
#include <WiFi.h>

DFRobot_AHT20 aht20;

// Struktura do wysyłki
struct Message {
  float temperature;
  float humidity;
};

// Adres MAC serwera – wpisz tutaj odbiornik!
uint8_t serverMac[] = {0xe4,0x65,0xb8,0x26,0x30,0x28};  

Message msg;

// Callback statusu wysyłki
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // inicjalizacja czujnika
  uint8_t status;
  while ((status = aht20.begin()) != 0) {
    Serial.print("AHT20 init failed. error: ");
    Serial.println(status);
    delay(1000);
  }

  // tryb stacji
  WiFi.mode(WIFI_STA);

  // init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // dodaj serwer jako peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Client ready...");
}

void loop() {
  if (aht20.startMeasurementReady(true)) {
    msg.temperature = aht20.getTemperature_C();
    msg.humidity = aht20.getHumidity_RH();

    Serial.print("Sending: ");
    Serial.print(msg.temperature);
    Serial.print(" °C, ");
    Serial.print(msg.humidity);
    Serial.println(" %RH");

    // wysyłka
    esp_now_send(serverMac, (uint8_t *)&msg, sizeof(msg));
  }
  delay(5000);
}
