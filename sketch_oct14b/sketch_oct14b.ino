#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <BaseStation.h>
#include <rbuffor.h>
#include <HiveFrames.h>
#include <RemoteNode.h>
#include <Bufor.h>
#include "setupWebServer.h"

// Konfiguracja Wi-Fi
const char* ssid = "Arudim";
const char* password = "pies1233";

// UART2 – RX2=16, TX2=17
HardwareSerial uart(2);

extern Bufor remotes[MAX_REMOTE];


BaseStation base;
uint8_t remoteCount = 0;

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
            remotes[frame.payload.hive.id-1].Add(node);
            
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
