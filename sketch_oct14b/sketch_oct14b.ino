#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include "rbuffor.h"
#include <HiveFrames.h>
#include "MAIN_html.h"

// Konfiguracja Wi-Fi
const char* ssid = "linksys__2";
const char* password = "tylkodladomu";

// UART2 – RX2=16, TX2=17
HardwareSerial uart(2);

#define MAX_REMOTE 4

// Struktury danych
struct RemoteNode {
  uint8_t id;
  uint8_t active;
  int16_t temperature;
  int16_t humidity;
  unsigned long timestamp;
  constexpr RemoteNode(uint8_t id_=0, uint8_t active_=0, int16_t temp_=0, int16_t hum_=0, unsigned long ts_=0)
    : id(id_), active(active_), temperature(temp_), humidity(hum_), timestamp(ts_) {}
};

constexpr RemoteNode dummy{};
rbufor<RemoteNode,10,dummy> remotes[MAX_REMOTE];
rbufor<RemoteNode,40,dummy> buforRotacyjny;

struct BaseStation {
  int16_t temperature;
  int16_t humidity;
  int16_t pressure;
  int16_t altitude;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};
BaseStation base;
uint8_t remoteCount = 0;

AsyncWebServer server(80);



void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Łączenie z WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
  }
  Serial.println(" Połączono!");
  Serial.println(WiFi.localIP());
  if(!MDNS.begin("ule")) {
    Serial.println("Błąd mDNS!");
  } else {
    Serial.println("http://ule.local");
  }
}

HiveFrame handleUART(){
  uint8_t buf[sizeof(HiveFrame)];
  unsigned long now = millis();
  unsigned long startTime = now;
  uint8_t size=0;
  uint8_t marker;
  HiveFrame frame;
  memset(buf,0,sizeof(buf));
  while (uart.available() >= 1 && (millis() - startTime < 50)) {
    marker = (uint8_t)uart.peek();
    if(marker==0x55){
      size=sizeof(HivePayload)+2;
    }
    if(marker==0x66){
      size=sizeof(MeteoPayload)+2;
    }
    //Serial.printf("marker:%x, %d\n",marker,size);
    if(size>0){
      int size_to_read=size;
      
      Serial.printf("read_data count %d\n", size);
      while(size_to_read>0)
      {
        while (uart.available() < size && (millis() - startTime < 100)){
        delay(1);
      }
        int read_bytes = uart.readBytes((uint8_t*)(buf+(size-size_to_read)),1);
        size_to_read-=read_bytes;
        Serial.printf("to read_data count %d\n", size-size_to_read);
        for (int i=0;i<sizeof(buf);i++){
          Serial.printf("%.2x,",buf[i]);
        }
        Serial.println("");
      }

      frame=HiveFrame(buf,size);
      Serial.println(frame.isHive()?"HIVE":"");
      Serial.println(frame.isMeteo()?"METEO":"");
      //frame.print();
    }
    else {
      uart.read(&marker,1);
    }

  }
  return frame;
}
void handleUART_() {
  unsigned long now = millis();
  unsigned long startTime = now;
  
  while (uart.available() >= 2 && (millis() - startTime < 50)) {
    int marker1 = uart.peek();
    
    if (marker1 == 0xAA) {
      if (uart.available() >= 8) {
        uart.read();
        if (uart.read() == 0x55) {
          uint8_t id = uart.read();
          uint8_t active = uart.read();
          int16_t temp, hum;
          uart.readBytes((char*)&temp, 2);
          uart.readBytes((char*)&hum, 2);
          if (id > 0 && id <= MAX_REMOTE) {
            RemoteNode node(id, active, temp, hum, now);
            remotes[id-1].dodaj(node);
            buforRotacyjny.dodaj(node);
          }
        }
        continue;
      } else {
        break;
      }
    }
    else if (marker1 == 0xBB) {
      int minBaseFrameLen = 2 + 14 + 1;
      if (uart.available() >= minBaseFrameLen) {
        uart.read();
        if (uart.read() == 0x66) {
          uart.readBytes((char*)&base.temperature, 2);
          uart.readBytes((char*)&base.humidity, 2);
          uart.readBytes((char*)&base.pressure, 2);
          uart.readBytes((char*)&base.altitude, 2);
          uart.readBytes((char*)&base.pm1_0, 2);
          uart.readBytes((char*)&base.pm2_5, 2);
          uart.readBytes((char*)&base.pm10, 2);
          remoteCount = uart.read();
          int neededRemoteBytes = remoteCount * 6;
          if (uart.available() >= neededRemoteBytes) {
            for (int i=0; i<remoteCount && i<MAX_REMOTE; ++i) {
              uint8_t id = uart.read();
              uint8_t active = uart.read();
              int16_t temp, hum;
              uart.readBytes((char*)&temp, 2);
              uart.readBytes((char*)&hum, 2);
              if (id > 0 && id <= MAX_REMOTE) {
                RemoteNode node(id, active, temp, hum, now);
                remotes[id-1].dodaj(node);
                buforRotacyjny.dodaj(node);
              }
            }
          }
        }
        continue;
      } else {
        break;
      }
    }
    else {
      uart.read();
    }
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", MAIN_html);
  });

  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest *req){
    DynamicJsonDocument doc(1024);
    doc["base"]["temperature"] = base.temperature / 100.0;
    doc["base"]["humidity"] = base.humidity / 100.0;
    doc["base"]["pressure"] = base.pressure / 10.0;
    doc["base"]["altitude"] = base.altitude ;
    doc["base"]["pm1_0"] = base.pm1_0;
    doc["base"]["pm2_5"] = base.pm2_5;
    doc["base"]["pm10"] = base.pm10;
    JsonArray nodes = doc.createNestedArray("remotes");
    for (int i = 0; i < MAX_REMOTE; i++) {
      bool wasData = remotes[i].size() > 0;
      RemoteNode obj = remotes[i].ostatni();
      if (wasData && obj.active) {
        JsonObject n = nodes.createNestedObject();
        n["id"] = obj.id;
        n["active"] = 1;
        n["temperature"] = obj.temperature / 100.0;
        n["humidity"] = obj.humidity / 10.0;
      }
    }
    String out; 
    serializeJson(doc, out);
    AsyncWebServerResponse *response = req->beginResponse(200, "application/json", out);
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    req->send(response);
  });

  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *req){
    if(!req->hasParam("id")){
      req->send(400, "application/json", "{\"error\":\"missing id\"}");
      return;
    }
    int ulId = req->getParam("id")->value().toInt();
    if(ulId < 1 || ulId > MAX_REMOTE){
      req->send(400, "application/json", "{\"error\":\"invalid id\"}");
      return;
    }
    
    DynamicJsonDocument doc(2048);
    JsonArray hist = doc.createNestedArray("history");
    unsigned long now = millis();
    
    remotes[ulId-1].reset();
    RemoteNode node;
    while((node = remotes[ulId-1].nastepny()).id != 0){
      JsonObject item = hist.createNestedObject();
      item["temperature"] = node.temperature / 100.0;
      item["humidity"] = node.humidity / 100.0;
      item["secondsAgo"] = (now - node.timestamp) / 1000; // Ile sekund temu
    }
    
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  uart.begin(19200, SERIAL_8N1, 16, 17);
  setupWiFi();
  setupWebServer();
}

void loop() {
  
  HiveFrame frame = handleUART();
  if(frame.isHive()){
            RemoteNode node(frame.payload.hive.id, frame.payload.hive.active, frame.payload.hive.local.temperature, frame.payload.hive.local.humidity, millis());
            remotes[frame.payload.hive.id-1].dodaj(node);
            buforRotacyjny.dodaj(node);
  }
  if(frame.isMeteo()){
          base.temperature=frame.payload.meteo.t_bme;
          base.humidity=frame.payload.meteo.h_bme;
          base.pressure=frame.payload.meteo.p_bme;
          base.altitude=frame.payload.meteo.a_bme;
          base.pm1_0=frame.payload.meteo.pm1_0;
          base.pm2_5=frame.payload.meteo.pm2_5;
          base.pm10=frame.payload.meteo.pm10;
  }
  delay(10);
}
