#include <ESP8266WiFi.h>
#include <espnow.h>

void setup() {

  Serial.begin(115200);
  
  // Needed to stop a boot loop
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv); 
}

void loop() {
  delay(1);
}

void OnDataRecv(unsigned char *mac, unsigned char *incomingData, unsigned char len) {
  unsigned char header[2];
  header[0]=27;
  header[1]=len;

  Serial.write(header,2);
  Serial.write(incomingData,len);
}
