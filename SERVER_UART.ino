#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "PMS.h"
#include <LiquidCrystal_I2C.h>

// ===== LCD =====
int lcdColumns = 20;
int lcdRows = 4;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  // deklaracja globalna

// ===== Rotacja ekranów =====
int displayIndex = 0; 
unsigned long lastUpdate = 0;
const unsigned long interval = 3000; // ms

// ===== Struktury danych =====
struct AHT20Message {
  float temperature;
  float humidity;
};

#define DEVICE_TIMEOUT_MS 5000
struct DeviceData {
  uint8_t mac[6];
  int id;
  float temperature;
  float humidity;
  unsigned long lastSeen;
  bool active;
};

std::vector<DeviceData> devices;
int nextID = 1;

// ===== Czujniki lokalne =====
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

HardwareSerial pmsSerial(2);
PMS pms(pmsSerial);
PMS::DATA pmsData;

// ===== Zmienne pomocnicze PMS5003 =====
int pm1_0 = 0;
int pm2_5 = 0;
int pm10  = 0;

// ===== Funkcje pomocnicze =====
bool compareMac(const uint8_t *mac1, const uint8_t *mac2) {
  return memcmp(mac1, mac2, 6) == 0;
}

int findDeviceIndex(const uint8_t *mac) {
  for (size_t i = 0; i < devices.size(); i++) {
    if (compareMac(devices[i].mac, mac)) return i;
  }
  return -1;
}

// Callback odbioru ESP-NOW 
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(AHT20Message)) {
    return;
  }

  AHT20Message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  const uint8_t *mac = info->src_addr;

  int idx = findDeviceIndex(mac);
  if (idx == -1) {
    DeviceData dev;
    memcpy(dev.mac, mac, 6);
    dev.id = nextID++;
    dev.temperature = msg.temperature;
    dev.humidity = msg.humidity;
    dev.lastSeen = millis();
    dev.active = true;
    devices.push_back(dev);
  } else {
    devices[idx].temperature = msg.temperature;
    devices[idx].humidity = msg.humidity;
    devices[idx].lastSeen = millis();
    devices[idx].active = true;
  }
}

// ===== Wysyłanie przez UART0 =====
unsigned long uartLastSend = 0;
const unsigned long uartInterval = 2000; // co 2s

void sendUartFrame() {
  uint8_t frame[128];
  int pos = 0;

  // Nagłówek
  frame[pos++] = 0xAA;
  frame[pos++] = 0x55;

  // Dane stacji bazowej
  int16_t t_bme = (int16_t)(bme.readTemperature() * 100);
  int16_t h_bme = (int16_t)(bme.readHumidity() * 100);
  int16_t p_bme = (int16_t)(bme.readPressure() / 10.0F); // <--- POPRAWIONE! hPa x10 (10132 dla 1013.2 hPa)
  int16_t a_bme = (int16_t)(bme.readAltitude(SEALEVELPRESSURE_HPA) * 100);
  uint16_t pm1 = (uint16_t)pm1_0;
  uint16_t pm25 = (uint16_t)pm2_5;
  uint16_t pm10v = (uint16_t)pm10;

  memcpy(frame+pos, &t_bme, 2);  pos+=2;
  memcpy(frame+pos, &h_bme, 2);  pos+=2;
  memcpy(frame+pos, &p_bme, 2);  pos+=2;
  memcpy(frame+pos, &a_bme, 2);  pos+=2;
  memcpy(frame+pos, &pm1, 2);    pos+=2;
  memcpy(frame+pos, &pm25, 2);   pos+=2;
  memcpy(frame+pos, &pm10v, 2);  pos+=2;

  // Liczba czujników zdalnych
  uint8_t count = devices.size() > 4 ? 4 : devices.size();
  frame[pos++] = count;

  // Dane do 4 czujników zdalnych
  unsigned long now = millis();
  for (int i=0; i<count; i++) {
    DeviceData &dev = devices[i];
    frame[pos++] = dev.id;
    frame[pos++] = (uint8_t)((now - dev.lastSeen < DEVICE_TIMEOUT_MS) ? 1 : 0);
    int16_t temp = (int16_t)(dev.temperature * 100);
    int16_t hum = (int16_t)(dev.humidity * 100);
    memcpy(frame+pos, &temp, 2); pos +=2;
    memcpy(frame+pos, &hum, 2);  pos +=2;
  }

  Serial.write(frame, pos); // wysyłka przez UART0 (TX0)
}


void setup() {
  Serial.begin(9600); // UART0, TX0=GPIO1, RX0=GPIO3
  WiFi.mode(WIFI_STA);

  lcd.init();
  lcd.backlight();

  if (esp_now_init() != ESP_OK) {
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);

  if (!bme.begin(0x76)) {
    while (1);
  }

  pmsSerial.begin(9600, SERIAL_8N1, 16, 17);
  pms.wakeUp();
  while (pmsSerial.available()) pmsSerial.read();
}

void loop() {
  // ===== Odczyty lokalne =====
  float t_bme = bme.readTemperature();
  float h_bme = bme.readHumidity();
  float p_bme = bme.readPressure() / 100.0F;
  float a_bme = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // ===== PMS5003 =====
  bool pmsReady = pms.read(pmsData);
  if (pmsReady) {
    pm1_0 = pmsData.PM_AE_UG_1_0;
    pm2_5 = pmsData.PM_AE_UG_2_5;
    pm10  = pmsData.PM_AE_UG_10_0;
  }

  // ===== LCD =====
  unsigned long now = millis();
  if (now - lastUpdate > interval) {
    lastUpdate = now;
    lcd.clear();
    delay(50);

    int totalScreens = 1 + devices.size();

    if (displayIndex == 0) {
      lcd.setCursor(0,0);
      lcd.print("Stacja bazowa");

      lcd.setCursor(0,1);
      lcd.print("T:");
      lcd.print(t_bme,1);
      lcd.print("C H:");
      lcd.print(h_bme,1);
      lcd.print("%");

      lcd.setCursor(0,2);
      lcd.print("P:");
      lcd.print(p_bme,0);
      lcd.print("hPa Alt:");
      lcd.print(a_bme,0);
      lcd.print("m");

      lcd.setCursor(0,3);
      lcd.print("PM2.5:");
      lcd.print(pm2_5);
      lcd.print(" PM10:");
      lcd.print(pm10);

    } else {
      DeviceData d = devices[displayIndex-1];

      lcd.setCursor(0,0);
      lcd.print("AHT20 zdalne");

      lcd.setCursor(0,1);
      lcd.print("ID:");
      lcd.print(d.id);

      lcd.setCursor(0,2);
      lcd.print("T:");
      lcd.print(d.temperature,1);
      lcd.print("C H:");
      lcd.print(d.humidity,1);
      lcd.print("%");

      lcd.setCursor(0,3);
      lcd.print("Stacja ");
      lcd.print(displayIndex);
      lcd.print("/");
      lcd.print(devices.size());
    }

    displayIndex++;
    if (displayIndex >= totalScreens) displayIndex = 0;
  }

  // ===== RAMKA UART0 =====
  if (millis() - uartLastSend > uartInterval) {
    sendUartFrame();
    uartLastSend = millis();
  }
}
