#include <esp_now.h>
#include <WiFi.h>

uint8_t reciever_macadr[] = {0xe4,0x65,0xb8,0x26,0x30,0x28};

typedef struct messege_struct{
  int x;
} messege_struct;

messege_struct myMessege;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status== ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup(){
  Serial.begin(9600);
  Serial.println("");
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, reciever_macadr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}
 
void loop(){

  myMessege.x = random(1,20);

  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(reciever_macadr, (uint8_t *) &myMessege, sizeof(myMessege));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}