// For the ethernet version build with Olimex ESP32 Gateway, rev F
// For the ethernet PoE version compile with Olimex ESP32-PoE
// For the DMX version build with node32s

#include <esp_now.h>
#include <WiFi.h>
#include "FastLED.h"

// #define SACN  1
#define ARTNET 1
// #define DMX 1

#ifdef SACN
#include <ESPAsyncE131.h>
ESPAsyncE131 e131(1);
#define ETHMODEL  1
#endif

#ifdef ARTNET
#include <ArtnetWifi.h>
ArtnetWifi artnet;
#define ETHMODEL  1
#endif

#ifdef DMX
#include <esp_dmx.h>

#define transmitPin 17
#define receivePin  16
#define enablePin   21
#define LED_PORT    2

dmx_port_t dmxPort = 1;

byte dmxdata[DMX_MAX_PACKET_SIZE];

QueueHandle_t queue;

#endif

#ifdef ETHMODEL
#include <ETH.h>
#include <ArduinoOTA.h>

// For ESP32-Gateway
// #define ETH_POWER_PORT  5
// #define LED_PORT    33

// For POE version
#define ETH_POWER_PORT  12
#define LED_PORT   33

static bool eth_connected = false;
unsigned long ethConnectedInitial=0;

#endif

// For sACN the start channel and universe numbers start at 1
// For the DMX and Artnet versions channel starts at 0
#define START_CHANNEL 1

// For the DMX version we have to set universe 0
#define UNIVERSE  2

#define NUM_PROPS_SINGLE  1
#define NUM_PROPS_MULTI    1

#define MULTI_OFFSET    (NUM_PROPS_SINGLE*sizeof(s_controlpacket_single))

uint8_t device_addresses_single[] = {
  0x84, 0x0D, 0x8E, 0x8C, 0xB4, 0xA8, \
};

uint8_t device_addresses_multi[] = {0x60, 0x01, 0x94, 0x6f, 0x4a, 0x07};

esp_now_peer_info_t slaves_single[NUM_PROPS_SINGLE];
esp_now_peer_info_t slaves_multi[NUM_PROPS_MULTI];

typedef struct t_outputdata {
  uint8_t patternmode;
  uint8_t patternvaluea;
  uint8_t patternvalueb;
  uint8_t glitter;
  CRGB startcolour;
  CRGB endcolour;
} s_outputdata;

typedef struct t_outputmap {
  uint8_t fader;
} s_outputmap;

typedef struct t_controlpacket_single {
  s_outputdata rx_outputdata[2];
  s_outputmap rx_outputmap[1];
} s_controlpacket_single;

typedef struct t_controlpacket_multi {
  s_outputdata rx_outputdata[2];
  s_outputmap rx_outputmap[8];
} s_controlpacket_multi;

s_controlpacket_single testpacket_single;
s_controlpacket_single prop_packets_single[NUM_PROPS_SINGLE];
s_controlpacket_single last_tx_single[NUM_PROPS_SINGLE];

s_controlpacket_multi testpacket_multi;
s_controlpacket_multi prop_packets_multi[NUM_PROPS_MULTI];
s_controlpacket_multi last_tx_multi[NUM_PROPS_MULTI];

int packet_acknowledgements_single[NUM_PROPS_SINGLE];
int packet_acknowledgements_multi[NUM_PROPS_MULTI];

#ifdef ETHMODEL
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("espnow_propTX");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      if (ethConnectedInitial==0) ethConnectedInitial=millis();
      if ((eth_connected==false)&&(millis()-ethConnectedInitial>10000)) {
        digitalWrite(ETH_POWER_PORT,false);
        Serial.print("Powering off LAN IC");
        for(int i=0;i<10;i++) {
          digitalWrite(LED_PORT,true);
          delay(100);
          digitalWrite(LED_PORT,false);
          delay(100);
        }
        Serial.print("Restarting");
        delay(100);
        ESP.restart();         
      }
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      ethConnectedInitial=0;     
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}
#endif

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  int i=0;
  int device_id=-1;
  while((i<NUM_PROPS_SINGLE)&&(device_id==-1)) {
    if (memcmp(&(slaves_single[i].peer_addr),mac_addr,6)==0) device_id=i; else i++;
  }
  if (device_id>=0) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      packet_acknowledgements_single[device_id]=0;
      memcpy(&(last_tx_single[device_id]),&(prop_packets_single[device_id]),sizeof(s_controlpacket_single));
    } else {
      packet_acknowledgements_single[device_id]=2;
    }
  } else {
    // Check the multis
    i=0;
    while((i<NUM_PROPS_MULTI)&&(device_id==-1)) {
      if (memcmp(&(slaves_multi[i].peer_addr),mac_addr,6)==0) device_id=i; else i++;
    }
    if (device_id>=0) {
      if (status == ESP_NOW_SEND_SUCCESS) {
        packet_acknowledgements_multi[device_id]=0;
        memcpy(&(last_tx_multi[device_id]),&(prop_packets_multi[device_id]),sizeof(s_controlpacket_multi));
      } else {
        packet_acknowledgements_multi[device_id]=2;
      }
    }
  }
}

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  int i;
  int retry;
  digitalWrite(LED_PORT,true);
  if (universe==UNIVERSE) {
    for(i=0;i<NUM_PROPS_SINGLE;i++) {
      if (memcmp(data+START_CHANNEL+i*sizeof(s_controlpacket_single),&(last_tx_single[i]),sizeof(s_controlpacket_single))!=0) {
        if (packet_acknowledgements_single[i]!=1) {
          packet_acknowledgements_single[i]=1;
          memcpy(&(prop_packets_single[i]),data+START_CHANNEL+i*sizeof(s_controlpacket_single),sizeof(s_controlpacket_single));
          retry=5;
          while ((--retry>0)&&(esp_now_send(slaves_single[i].peer_addr, (uint8_t *) &(prop_packets_single[i]), sizeof(s_controlpacket_single))!=ESP_OK)) {
            delay(3);
          }
        }
      }
    }
    for(i=0;i<NUM_PROPS_MULTI;i++) {
      if (memcmp(data+START_CHANNEL+MULTI_OFFSET+i*sizeof(s_controlpacket_multi),&(last_tx_multi[i]),sizeof(s_controlpacket_multi))!=0) {
        if (packet_acknowledgements_multi[i]!=1) {
          packet_acknowledgements_multi[i]=1;
          memcpy(&(prop_packets_multi[i]),data+START_CHANNEL+MULTI_OFFSET+i*sizeof(s_controlpacket_multi),sizeof(s_controlpacket_multi));
          retry=5;
          while ((--retry>0)&&(esp_now_send(slaves_multi[i].peer_addr, (uint8_t *) &(prop_packets_multi[i]), sizeof(s_controlpacket_multi))!=ESP_OK)) {
            delay(3);
          }
        }
      }
    }
  }
  digitalWrite(LED_PORT,false);
}

void setup() {
  int i;
  
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(LED_PORT,OUTPUT);
  digitalWrite(LED_PORT,false);

#ifdef ETHMODEL
  pinMode(ETH_POWER_PORT,OUTPUT);
  digitalWrite(ETH_POWER_PORT,false);
  delay(250);
  digitalWrite(ETH_POWER_PORT,true);
  delay(250);
  digitalWrite(LED_PORT,true);

  // Have to use these specific pin assignments on the ESP32-Gateway board
  ETH.begin(0, -1, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);  
  ETH.config(IPAddress(10, 0, 3, 75),IPAddress(10, 0, 3, 1),IPAddress(255, 255, 255, 0));

  ArduinoOTA.setHostname("propnet");
  ArduinoOTA.begin();

#endif
  
  
  // Needed to stop a boot loop
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if (NUM_PROPS_SINGLE>0) {
    memset(prop_packets_single,0,sizeof(prop_packets_single));
    memset(last_tx_single,0,sizeof(last_tx_single));
    memset(packet_acknowledgements_single,0,sizeof(packet_acknowledgements_single));

    testpacket_single.rx_outputdata[0].patternmode=0;
    testpacket_single.rx_outputdata[0].startcolour=CRGB::Blue;
    
  }

  if (NUM_PROPS_MULTI>0) {
    memset(prop_packets_multi,0,sizeof(prop_packets_multi));
    memset(last_tx_multi,0,sizeof(last_tx_multi));
    memset(packet_acknowledgements_multi,0,sizeof(packet_acknowledgements_multi));
    testpacket_multi.rx_outputdata[0].patternmode=0;
    testpacket_multi.rx_outputdata[0].startcolour=CRGB::Blue;
  }

  // esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  for(i=0;i<NUM_PROPS_SINGLE;i++) {
    memcpy( &(slaves_single[i].peer_addr), device_addresses_single+i*6, 6 );
    esp_now_add_peer(&slaves_single[i]);
    delay(10);
    esp_now_send(slaves_single[i].peer_addr, (uint8_t *) &testpacket_single, sizeof(testpacket_single));
    delay(100);
  }

  for(i=0;i<NUM_PROPS_MULTI;i++) {
    memcpy( &(slaves_multi[i].peer_addr), device_addresses_multi+i*6, 6 );
    esp_now_add_peer(&slaves_multi[i]);
    delay(10);
    esp_now_send(slaves_multi[i].peer_addr, (uint8_t *) &testpacket_multi, sizeof(testpacket_multi));
    delay(100);
  }

  // Start DMX receiption
  
 #ifdef SACN
  e131.begin(E131_MULTICAST, UNIVERSE, 1);
 #endif
  
 #ifdef ARTNET
  artnet.begin();
  artnet.setArtDmxCallback(onDmxFrame);
 #endif

 #ifdef DMX
  dmx_config_t dmxConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxPort, &dmxConfig);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  int queueSize = 1;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, &queue,
                     interruptPriority);
 #endif  

  digitalWrite(LED_PORT,false);

}
 
void loop() {
#ifdef ETHMODEL
  ArduinoOTA.handle();  
#endif
#ifdef SACN
  if (!e131.isEmpty()) {
      e131_packet_t packet;
      e131.pull(&packet);
      onDmxFrame(htons(packet.universe),htons(packet.property_value_count),e131.stats.num_packets&0xff,packet.property_values);
  }
#endif
#ifdef ARTNET
  artnet.read();
#endif
#ifdef DMX
  dmx_event_t packet;
  static unsigned char lastdata=0;
  if (xQueueReceive(queue, &packet, DMX_PACKET_TIMEOUT_TICK)) {
    if (packet.status == DMX_OK) {
      dmx_read_packet(dmxPort, dmxdata, packet.size);
      onDmxFrame(0,packet.size,0,dmxdata);
    }
  }
#endif
  delay(1);
}
