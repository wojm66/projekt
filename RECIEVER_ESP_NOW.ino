#include <esp_now.h>
#include <WiFi.h>

#define MAX_DEV 10

typedef struct messege_struct{
  int x;
} messege_struct;

messege_struct myMessege;

uint8_t knownDevices[MAX_DEV] [6];
int devKnown = 0;

bool isNewDev(const uint8_t *mac){
  for(int i=0; i<devKnown; i++){
    if(memcmp(mac, knownDevices[i], 6)==0){
      return false;
    }
  }

  if(devKnown<MAX_DEV){
    memcpy(knownDevices[devKnown], mac,6);
    devKnown++;
  }
  return true;
}

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myMessege, incomingData, sizeof(myMessege));
  Serial.print("Int: ");
  Serial.println(myMessege.x);

  if (isNewDev(mac)){
    Serial.print("New device detected: ");
    printMac(mac);
    Serial.println();
  }
}

void setup() {
    Serial.begin(9600);
  Serial.println("\n");
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else{
    Serial.println("Succes initializing ESP-NOW");
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop(){

}
