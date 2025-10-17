#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

// Konfiguracja Wi-Fi
const char* ssid = "TP-Link_57B4_16";
const char* password = "5m3FkT8JoHsMFrj";

// UART2 – RX2=16, TX2=17
HardwareSerial uart(2);

#define MAX_REMOTE 4

// Struktury danych
struct RemoteNode {
  uint8_t id;
  uint8_t active;
  int16_t temperature;
  int16_t humidity;
};

struct BaseStation {
  int16_t temperature;
  int16_t humidity;
  int16_t pressure;
  int16_t altitude;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};

RemoteNode remotes[MAX_REMOTE];
BaseStation base;
uint8_t remoteCount = 0;

AsyncWebServer server(80);

// ---------------- STRONA HTML + JS -----------------
const char MAIN_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<title>Monitoring Pasieki IoT</title>
<style>
body{
  margin:0; font-family:'Segoe UI',Arial,sans-serif; background:#f2f5ed; color:#333;
}
header{
  background:linear-gradient(135deg,#6B8E23,#3a5515); color:white; padding:20px 30px;
  box-shadow:0 2px 6px rgba(0,0,0,0.2);
}
header h1{margin:0; font-size:24px;}
.grid{
  display:grid; grid-template-columns:repeat(auto-fit,minmax(290px,1fr));
  gap:20px; padding:20px;
}
.card{
  background:white; border-radius:10px; box-shadow:0 2px 5px rgba(0,0,0,0.1);
  padding:18px; transition:transform .2s;
}
.card:hover{ transform:scale(1.02);}
.card h2{
  font-size:18px; color:#3a5515; border-bottom:1px solid #ccc; padding-bottom:5px;
}
.value{
  font-size:22px; font-weight:bold; color:#222;
}
.label{
  font-size:13px; color:#666;
}
.remote{
  background:#fafafa; border:1px solid #dcdcdc; border-radius:8px;
  padding:10px; margin-top:10px;
}
.remote.active{ border-left:5px solid #6B8E23;}
.remote.inactive{ border-left:5px solid #888;}
footer{
  text-align:center; font-size:12px; color:#666; padding:10px;
}
</style>
</head>
<body>
<header>
  <h1>📡 Monitoring Pasieki IoT – Ule i Otoczenie</h1>
  <p>Podgląd w czasie rzeczywistym: <b>http://stacja.local</b></p>
</header>

<div class="grid">
  <div class="card">
    <h2>Dane środowiskowe – Pasieka</h2>
    <div><span class="label">Temperatura:</span> <span id="base-temp" class="value"></span> °C</div>
    <div><span class="label">Wilgotność:</span> <span id="base-hum" class="value"></span> %</div>
    <div><span class="label">Ciśnienie:</span> <span id="base-pressure" class="value"></span> hPa</div>
    <div><span class="label">Wysokość:</span> <span id="base-alt" class="value"></span> m</div>
  </div>

  <div class="card">
    <h2>Jakość powietrza</h2>
    <div><span class="label">PM1.0:</span> <span id="base-pm1" class="value"></span> µg/m³</div>
    <div><span class="label">PM2.5:</span> <span id="base-pm25" class="value"></span> µg/m³</div>
    <div><span class="label">PM10:</span> <span id="base-pm10" class="value"></span> µg/m³</div>
  </div>

  <div class="card">
    <h2>Czujniki w ulach</h2>
    <div id="remote-1" class="remote"></div>
    <div id="remote-2" class="remote"></div>
    <div id="remote-3" class="remote"></div>
    <div id="remote-4" class="remote"></div>
  </div>
</div>

<footer>🐝 Projekt pasieki IoT – monitorowanie mikroklimatu uli i otoczenia • ESP32 & Wi-Fi</footer>

<script>
function updatePage(data){
  // Dane stacji bazowej
  document.getElementById("base-temp").innerText=data.base.temperature.toFixed(1);
  document.getElementById("base-hum").innerText=data.base.humidity.toFixed(1);
  document.getElementById("base-pressure").innerText=data.base.pressure.toFixed(1);
  document.getElementById("base-alt").innerText=data.base.altitude.toFixed(1);
  document.getElementById("base-pm1").innerText=data.base.pm1_0;
  document.getElementById("base-pm25").innerText=data.base.pm2_5;
  document.getElementById("base-pm10").innerText=data.base.pm10;

  // Dane z czujników
  for(let i=0;i<4;i++){
    let n=data.remotes[i], el=document.getElementById("remote-"+(i+1));
    if(!n) continue;
    el.className=n.active?"remote active":"remote inactive";
    if(n.active){
      el.innerHTML=`<b>Ul ${n.id}</b> – aktywny<br>
        🌡️ Temp: ${n.temperature.toFixed(1)} °C<br>
        💧 Wilgotność: ${n.humidity.toFixed(1)} %`;
    }else{
      el.innerHTML=`<b>Ul ${n.id}</b> – nieaktywny<br>Brak danych`;
    }
  }
}
function getData(){ fetch("/data.json").then(r=>r.json()).then(updatePage);}
setInterval(getData,2000);
window.onload=getData;
</script>
</body>
</html>
)rawliteral";


// ---------------- FUNKCJE -----------------
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Łączenie z WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Połączono!");
  Serial.println(WiFi.localIP());

  if(!MDNS.begin("stacja")) {
    Serial.println("Błąd mDNS!");
  } else {
    Serial.println("http://stacja.local");
  }
}

void handleUART() {
  int minFrame = 2 + 14 + 1 + MAX_REMOTE * 6;
  if (uart.available() >= minFrame) {
    uint8_t buf[64];
    int len = uart.readBytes(buf, sizeof(buf));
    int pos = 0;
    while (pos+1 < len && !(buf[pos] == 0xAA && buf[pos+1] == 0x55)) pos++;
    if (pos+1 >= len) return;
    pos += 2;
    memcpy(&base.temperature, buf+pos, 2); pos+=2;
    memcpy(&base.humidity, buf+pos, 2); pos+=2;
    memcpy(&base.pressure, buf+pos, 2); pos+=2;
    memcpy(&base.altitude, buf+pos, 2); pos+=2;
    memcpy(&base.pm1_0, buf+pos, 2); pos+=2;
    memcpy(&base.pm2_5, buf+pos, 2); pos+=2;
    memcpy(&base.pm10, buf+pos, 2); pos+=2;

    remoteCount = buf[pos++];
    for (int i=0;i<remoteCount && i<MAX_REMOTE;i++){
      remotes[i].id = buf[pos++];
      remotes[i].active = buf[pos++];
      memcpy(&remotes[i].temperature, buf+pos, 2); pos+=2;
      memcpy(&remotes[i].humidity, buf+pos, 2); pos+=2;
    }

    // Debug
    Serial.println("===== Odczyt =====");
    Serial.printf("T: %.2f°C H: %.2f%% P: %.1f hPa A: %.2f m\n",
                  base.temperature/100.0, base.humidity/100.0,
                  base.pressure/10.0, base.altitude/100.0);
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", MAIN_html);
  });

  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest *req){
    DynamicJsonDocument doc(1024);
    doc["base"]["temperature"] = base.temperature/100.0;
    doc["base"]["humidity"] = base.humidity/100.0;
    doc["base"]["pressure"] = base.pressure/10.0;
    doc["base"]["altitude"] = base.altitude/100.0;
    doc["base"]["pm1_0"] = base.pm1_0;
    doc["base"]["pm2_5"] = base.pm2_5;
    doc["base"]["pm10"] = base.pm10;
    JsonArray nodes = doc.createNestedArray("remotes");
    for (int i=0;i<MAX_REMOTE;i++){
      JsonObject n = nodes.createNestedObject();
      n["id"] = remotes[i].id;
      n["active"] = remotes[i].active;
      n["temperature"] = remotes[i].temperature/100.0;
      n["humidity"] = remotes[i].humidity/100.0;
    }
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.begin();
}

// ---------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(115200);
  uart.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("UART Odbiornik uruchomiony");

  setupWiFi();
  setupWebServer();
}

void loop() {
  handleUART();
  delay(400);
}
