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
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  // <-- deklaracja globalna

// ===== Rotacja ekranów =====
int displayIndex = 0; 
unsigned long lastUpdate = 0;
const unsigned long interval = 3000; // ms

// ===== Struktury danych =====
struct AHT20Message {
  float temperature;
  float humidity;
};

struct DeviceData {
  uint8_t mac[6];
  int id;
  float temperature;
  float humidity;
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
    Serial.println("Błędny rozmiar pakietu");
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
    devices.push_back(dev);

    Serial.print("Nowe urządzenie AHT20 (ID ");
    Serial.print(dev.id);
    Serial.println(") dodane");
  } else {
    devices[idx].temperature = msg.temperature;
    devices[idx].humidity = msg.humidity;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // ===== LCD =====
  lcd.init();
  lcd.backlight();

  // ===== ESP-NOW =====
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);

  // ===== BME280 =====
  if (!bme.begin(0x76)) {
    Serial.println("Nie znaleziono BME280!");
    while (1);
  }

  // ===== PMS5003 =====
  pmsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  pms.wakeUp();
  while (pmsSerial.available()) pmsSerial.read();

  Serial.println("Serwer gotowy...");
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

    int totalScreens = 1 + devices.size(); // 1 ekran stacja bazowa + N zdalnych

    if (displayIndex == 0) {
      // --- Stacja bazowa ---
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
      // --- Zdalna stacja ---
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

  // ===== Serial Monitor =====
  Serial.println("=== Odczyty lokalne ===");
  Serial.print("BME280 -> Temp: ");
  Serial.print(t_bme,1);
  Serial.print(" °C | Hum: ");
  Serial.print(h_bme,1);
  Serial.print(" % | Press: ");
  Serial.print(p_bme,1);
  Serial.print(" hPa | Alt: ");
  Serial.print(a_bme,1);
  Serial.println(" m");

  Serial.print("PMS5003 -> PM1.0: ");
  Serial.print(pm1_0);
  Serial.print(" | PM2.5: ");
  Serial.print(pm2_5);
  Serial.print(" | PM10: ");
  Serial.println(pm10);

  if (!devices.empty()) {
    Serial.println("--- Odczyty AHT20 ---");
    for (auto &d : devices) {
      Serial.print("ID ");
      Serial.print(d.id);
      Serial.print(" | Temp: ");
      Serial.print(d.temperature,1);
      Serial.print(" °C | Hum: ");
      Serial.print(d.humidity,1);
      Serial.println(" %");
    }
  }
  Serial.println("-------------------------");

  delay(2000); // dostosuj do potrzeb
}
