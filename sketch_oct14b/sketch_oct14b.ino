#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include "rbuffor.h"

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

const char MAIN_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Monitoring Pasieki IoT</title>
<style>
body{margin:0;font-family:'Segoe UI',Arial,sans-serif;background:#f2f5ed;color:#333;}
header{background:linear-gradient(135deg,#6B8E23,#3a5515);color:white;padding:20px 30px;box-shadow:0 2px 6px rgba(0,0,0,0.2);}
header h1{margin:0;font-size:24px;}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:20px;padding:20px;}
.card{background:white;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,0.1);padding:18px;transition:transform .2s;}
.card:hover{transform:scale(1.02);}
.card h2{font-size:18px;color:#3a5515;border-bottom:1px solid #ccc;padding-bottom:5px;margin-bottom:10px;}
.value{font-size:22px;font-weight:bold;color:#222;}
.label{font-size:13px;color:#666;}
.remote{background:#fafafa;border:1px solid #dcdcdc;border-radius:8px;padding:10px;margin-top:10px;cursor:pointer;transition:background .2s;}
.remote:hover{background:#e8f5e9;}
.remote.active{border-left:5px solid #6B8E23;}
footer{text-align:center;font-size:12px;color:#666;padding:10px;}
#modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);z-index:1000;align-items:center;justify-content:center;}
#modal-content{background:white;border-radius:10px;padding:20px;max-width:600px;width:90%;max-height:80vh;overflow-y:auto;}
#modal-close{float:right;font-size:24px;cursor:pointer;color:#666;}
.history-item{padding:10px;border-bottom:1px solid #eee;}
</style>
</head>
<body>
<header>
  <h1>📡 Monitoring Pasieki IoT – Ule i Otoczenie</h1>
  <p>Podgląd w czasie rzeczywistym: <b>http://ule.local</b></p>
</header>
<div class="grid">
  <div class="card">
    <h2>Dane środowiskowe – Pasieka</h2>
    <div><span class="label">Temperatura:</span> <span id="base-temp" class="value">--</span> °C</div>
    <div><span class="label">Wilgotność:</span> <span id="base-hum" class="value">--</span> %</div>
    <div><span class="label">Ciśnienie:</span> <span id="base-pressure" class="value">--</span> hPa</div>
    <div><span class="label">Wysokość:</span> <span id="base-alt" class="value">--</span> m</div>
  </div>
  <div class="card">
    <h2>Jakość powietrza</h2>
    <div><span class="label">PM1.0:</span> <span id="base-pm1" class="value">--</span> µg/m³</div>
    <div><span class="label">PM2.5:</span> <span id="base-pm25" class="value">--</span> µg/m³</div>
    <div><span class="label">PM10:</span> <span id="base-pm10" class="value">--</span> µg/m³</div>
  </div>
  <div class="card">
    <h2>Czujniki w ulach</h2>
    <div id="remotes-container"></div>
  </div>
</div>
<div id="modal">
  <div id="modal-content">
    <span id="modal-close">&times;</span>
    <h2 id="modal-title">Historia Ula</h2>
    <div id="modal-history"></div>
  </div>
</div>
<footer>🐝 Projekt pasieki IoT – monitorowanie mikroklimatu uli i otoczenia • ESP32 & Wi-Fi</footer>
<script>
function updatePage(data){
  document.getElementById("base-temp").innerText=data.base.temperature.toFixed(1);
  document.getElementById("base-hum").innerText=data.base.humidity.toFixed(1);
  document.getElementById("base-pressure").innerText=data.base.pressure.toFixed(1);
  document.getElementById("base-alt").innerText=data.base.altitude.toFixed(1);
  document.getElementById("base-pm1").innerText=data.base.pm1_0;
  document.getElementById("base-pm25").innerText=data.base.pm2_5;
  document.getElementById("base-pm10").innerText=data.base.pm10;
  
  let container = document.getElementById('remotes-container');
  container.innerHTML = '';
  
  let activeCount = 0;
  for(let i=0; i<data.remotes.length; i++){
    let n = data.remotes[i];
    if(!n || !n.active) continue;
    activeCount++;
    
    let div = document.createElement('div');
    div.className = 'remote active';
    div.onclick = () => showHistory(n.id);
    div.innerHTML = `<b>Ul ${n.id}</b> – aktywny<br>
      🌡️ Temp: ${n.temperature.toFixed(1)} °C<br>
      💧 Wilgotność: ${n.humidity.toFixed(1)} %`;
    container.appendChild(div);
  }
  
  if(activeCount === 0){
    container.innerHTML = '<p style="color:#999;">Brak aktywnych uli</p>';
  }
}

function showHistory(ulId){
  fetch('/history?id=' + ulId)
    .then(r => r.json())
    .then(data => {
      document.getElementById('modal-title').innerText = 'Historia Ula ' + ulId;
      let hist = document.getElementById('modal-history');
      hist.innerHTML = '';
      if(data.history && data.history.length > 0){
        data.history.forEach(item => {
          let timeStr = formatTime(item.secondsAgo);
          let div = document.createElement('div');
          div.className = 'history-item';
          div.innerHTML = `<span style="color:#888;">${timeStr}</span><br>
            🌡️ ${item.temperature.toFixed(1)} °C | 💧 ${item.humidity.toFixed(1)} %`;
          hist.appendChild(div);
        });
      } else {
        hist.innerHTML = '<p>Brak danych historycznych</p>';
      }
      document.getElementById('modal').style.display = 'flex';
    })
    .catch(err => console.error('Błąd pobierania historii:', err));
}

function formatTime(secondsAgo){
  if(secondsAgo < 60) return secondsAgo + ' sek. temu';
  let minutes = Math.floor(secondsAgo / 60);
  if(minutes < 60) return minutes + ' min. temu';
  let hours = Math.floor(minutes / 60);
  if(hours < 24) return hours + ' godz. temu';
  let days = Math.floor(hours / 24);
  return days + ' dni temu';
}

document.getElementById('modal-close').onclick = () => {
  document.getElementById('modal').style.display = 'none';
};

document.getElementById('modal').onclick = (e) => {
  if(e.target.id === 'modal') document.getElementById('modal').style.display = 'none';
};

function getData(){ 
  fetch("/data.json")
    .then(r => r.json())
    .then(updatePage)
    .catch(err => console.error("Błąd pobierania danych:", err));
}
setInterval(getData, 2000);
window.onload = getData;
</script>
</body>
</html>
)rawliteral";

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

void handleUART() {
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
    doc["base"]["altitude"] = base.altitude / 100.0;
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
        n["humidity"] = obj.humidity / 100.0;
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
  uart.begin(9600, SERIAL_8N1, 16, 17);
  setupWiFi();
  setupWebServer();
}

void loop() {
  handleUART();
  delay(10);
}
