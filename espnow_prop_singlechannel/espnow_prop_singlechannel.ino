// Compile using LOLIN(WEMOS) D1 R2 and mini
#include <ESP8266WiFi.h>
#include <espnow.h>

#define NUM_CHANNELS  3
const int ChannelMap[] = { D5, D6, D7 };

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0;i<NUM_CHANNELS;i++) {
    pinMode(ChannelMap[i], OUTPUT); // The main output pin
    analogWrite(ChannelMap[i],50); // Dim glow
  }

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
  digitalWrite(LED_BUILTIN, HIGH);
  if (len==21) {
    for(int i=0;i<NUM_CHANNELS;i++) {
      analogWrite(ChannelMap[i],incomingData[i]*4);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}
