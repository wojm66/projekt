#include "setupWebServer.h"
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <BaseStation.h>
#include <RemoteNode.h>
#include <rbuffor.h>
#include <Bufor.h>
#include "MAIN_html.h"


Bufor remotes[MAX_REMOTE];
extern BaseStation base;

AsyncWebServer server(80);

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
      //Serial.printf("i: %d, c: %d\n", i, remotes[i].Size());
      bool wasData = remotes[i].Size() > 0;
      RemoteNode obj = remotes[i].Last();
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
    Serial.printf("Ul id : %d\n",ulId);
    if(ulId < 1 || ulId > MAX_REMOTE){
      req->send(400, "application/json", "{\"error\":\"invalid id\"}");
      return;
    }
    
    DynamicJsonDocument doc(2048);
    JsonArray hist = doc.createNestedArray("history");
    unsigned long now = millis();
    
    remotes[ulId-1].Reset();
    RemoteNode node;
    while( (node = remotes[ulId-1].Next()).id != 0 ){
      JsonObject item = hist.createNestedObject();
      item["temperature"] = node.temperature / 100.0;
      item["humidity"] = node.humidity / 10.0;
      item["secondsAgo"] = (now - node.timestamp) / 1000; // Ile sekund temu
    }
    
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.begin();
}