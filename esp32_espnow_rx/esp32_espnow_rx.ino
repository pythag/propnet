// Compile using LOLIN(WEMOS) D1 R2 mini or Arduino mBed Raspberry Pi Pico (for 8 way box)
// For the dice (and perhaps other 18650 based units) need to use Generic 8266 module with 2MB of flash

const char* ssid     = "";
const char* password = "";

// #define SERIALBRIDGEVERSION

#ifdef ESP32

// #define FASTLED_ALLOW_INTERRUPTS 0
// #define FASTLED_INTERRUPT_RETRY_COUNT 0
// #define FASTLED_ESP32_I2S 

// static TaskHandle_t FastLEDshowTaskHandle = 0;
// static TaskHandle_t userTaskHandle = 0;

#ifndef SERIALBRIDGEVERSION

// For node32s
#include <esp_now.h>
#include <WiFi.h>

#endif // SERIALBRIDGEVERSION
#endif // ESP32

#ifdef ESP8266

#ifndef SERIALBRIDGEVERSION

// For D1 R2 minis
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ArduinoOTA.h>

#endif // SERIALBRIDGEVERSION
#endif // ESP8266

#include <SPI.h>
#include "FastLED.h"

FASTLED_USING_NAMESPACE

// The single chains
/*
// Settings for the hoop
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    84
#define NUM_STRINGS 1
*/

/*
// Settings for the clover
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    16
#define NUM_STRINGS 1
*/

/*
// Settings for the long poles
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    60
#define NUM_STRINGS 1
*/

/*
// Settings for the short poles
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    30
#define NUM_STRINGS 1
*/

/*
// Settings for the 8-way box
#define EIGHTWAY    1
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    150
#define NUM_STRINGS 8
*/

/*
// Settings for the arrow
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    19
#define NUM_STRINGS 1
*/

/*
// Settings for a full string
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    150
#define NUM_STRINGS 1
*/

// Settings for a the dice
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
#define NUM_LEDS    21
#define NUM_STRINGS 1
#define DICE_MODE   1

// After 5 minutes of no ESPNOW signal being received connect to wifi
#define OTA_TIMEOUT 300
int allow_transition_to_ota=1;

#ifndef D2
#define D2  4
#endif

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

typedef struct t_controlpacket_multi {
  s_outputdata rx_outputdata[2];
  s_outputmap rx_outputmap[NUM_STRINGS];
} s_controlpacket;

s_controlpacket datapacket;

CRGB ledrendered[NUM_LEDS*2];
CRGB ledoutput[NUM_LEDS*NUM_STRINGS];
CRGB BlackLed = CRGB::Black;

char needs_constant_refresh=0;
char refresh_required=0;

#ifdef SERIALBRIDGEVERSION

char serial_state=0;
unsigned char bytes_expected=0;
unsigned char bytes_received=0;
unsigned char serialrxbuffer[256];

#endif

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND   100

CRGB FadeColour(CRGB source, CRGB dest, int percentage)
{
  int r,g,b;
  if (percentage>100) percentage=100;
  if (percentage<0) percentage=0;
  r=(((dest.r-source.r)*percentage)/100)+source.r;
  g=(((dest.g-source.g)*percentage)/100)+source.g;
  b=(((dest.b-source.b)*percentage)/100)+source.b;
 
  return CRGB(r,g,b);
}

void FadeColourInt(CRGB *output, CRGB *source, CRGB *dest, uint8_t ratio)
{
  output->r=((((int)(dest->r)-(int)(source->r))*ratio)/255)+source->r;
  output->g=((((int)(dest->g)-(int)(source->g))*ratio)/255)+source->g;
  output->b=((((int)(dest->b)-(int)(source->b))*ratio)/255)+source->b;
}

/*
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) {
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );       
        userTaskHandle = xTaskGetCurrentTaskHandle();
        xTaskNotifyGive(FastLEDshowTaskHandle);
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS( 100 ));     
        userTaskHandle = 0;
    }
}

void FastLEDshowTask(void *pvParameters)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
    for(;;) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        // noInterrupts();
        FastLED.show();
        // interrupts();
        xTaskNotifyGive(userTaskHandle);
    }
}
*/

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef ESP8266

  // Needed to stop a boot loop
  WiFi.mode(WIFI_STA);

#endif

#ifdef ESP32

  // Needed to stop a boot loop
  WiFi.mode(WIFI_STA);

#endif

#ifdef SERIALBRIDGEVERSION

#ifdef ESP32
  Serial2.begin(115200, SERIAL_8N1, 35, -1);  
#endif

#ifdef ARDUINO_ARCH_RP2040
  Serial1.begin(115200);
#endif

#else
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println(WiFi.macAddress());
  Serial.println("ESP-NOW Listening");  
#endif  
  
  // Setup the LED stuff
#ifdef ARDUINO_ARCH_RP2040
  FastLED.addLeds<LED_TYPE,28,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,27,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,26,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,22,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,21,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,20,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,19,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,18,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
#endif

#ifdef ESP32
  FastLED.addLeds<LED_TYPE,D2,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
#endif

#ifdef ESP8266
  FastLED.addLeds<LED_TYPE,D2,COLOR_ORDER>(ledoutput, NUM_LEDS).setCorrection(TypicalLEDStrip);
#endif

  FastLED.setDither( 0 );
  FastLED.clear();
  FastLED.show();
  fill_rainbow(ledoutput,NUM_LEDS*NUM_STRINGS,0,5);
  FastLED.show();

/*
  xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 16000, NULL,2, &FastLEDshowTaskHandle, 0);
*/
 
}

void loop() {

  static unsigned long nextrefresh=0;

  if ((refresh_required||needs_constant_refresh)&&(millis()>nextrefresh)) {
    refresh_required=0;
    nextrefresh=millis()+20;
    refreshoutput();
  }

#ifdef ESP8266

  if (allow_transition_to_ota==1) {
    if (millis()>OTA_TIMEOUT*1000) {
      // Transition into OTA mode
      allow_transition_to_ota=2;
      needs_constant_refresh=0;      
      refresh_required=0;
      fill_solid(ledoutput,NUM_LEDS*NUM_STRINGS,CRGB(10,0,0));
      FastLED.show();
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
    }
  }

  if (allow_transition_to_ota==2) {
    if (WiFi.status() == WL_CONNECTED) {
        allow_transition_to_ota=3;
        fill_solid(ledoutput,NUM_LEDS*NUM_STRINGS,CRGB(0,10,0));
        FastLED.show();
        ArduinoOTA.setHostname("dice1");

        ArduinoOTA.onEnd([]() {
          fill_solid(ledoutput,NUM_LEDS*NUM_STRINGS,CRGB(10,0,10));
          FastLED.show();
          WiFi.disconnect(true);          
        });
        
        ArduinoOTA.begin();      
    }
  }
  
  if (allow_transition_to_ota==3) {
      ArduinoOTA.handle();  
  }

#endif
  
#ifdef SERIALBRIDGEVERSION
  int c=0;

  do {

#ifdef ESP32
    c=Serial2.read();
#endif

#ifdef ARDUINO_ARCH_RP2040
    c=Serial1.read();
#endif

    if (c>=0) {
      switch (serial_state) {
        case 0: // Waiting for start character
                if (c==27) {
                  serial_state=1;
                  digitalWrite(LED_BUILTIN, HIGH);
                }
                break;
        case 1: // Waiting for length byte
                bytes_expected=c;
                bytes_received=0;
                if (c>0) serial_state=2; else serial_state=0;
                break;
        case 2: // Receiving data
                serialrxbuffer[bytes_received]=c;
                if (++bytes_received==bytes_expected) {
                  serial_state=0;
                  if (bytes_expected==sizeof(s_controlpacket)) {
                    memcpy(&datapacket,serialrxbuffer,sizeof(s_controlpacket));
                    refresh_required=1;
                  }
                  digitalWrite(LED_BUILTIN, LOW);
                }
                break;             
      }
    }
  } while((c>=0)&&(serial_state>0));
#endif
  
}

void refreshoutput()
{
  int i=0;
  int t=0;
  int x=0;
  int y=0;
  CRGB tcol;
  float m;
  needs_constant_refresh=0;
  for(i=0;i<2;i++) {
    switch(datapacket.rx_outputdata[i].patternmode/37) {
      case 0: // All the same colour
              fill_solid(ledrendered+(NUM_LEDS*i),NUM_LEDS,datapacket.rx_outputdata[i].startcolour);
              break;
      case 1: // Linear Gradient
              for(t=0;t<NUM_LEDS;t++) {
                ledrendered[i*NUM_LEDS+t]=FadeColour(datapacket.rx_outputdata[i].startcolour,datapacket.rx_outputdata[i].endcolour,(t*100)/NUM_LEDS);
              }
              break;
      case 2: // Gradient with sine wave
              m=datapacket.rx_outputdata[i].patternvaluea/25.0;
              for(t=0;t<NUM_LEDS;t++) {
                ledrendered[i*NUM_LEDS+t]=FadeColour(datapacket.rx_outputdata[i].startcolour,datapacket.rx_outputdata[i].endcolour,abs(sin((datapacket.rx_outputdata[i].patternvalueb*3.141/255.0)+((3.141*t*m)/NUM_LEDS)))*100.0);
              }
              break;
      case 3: // Alternate
              for(t=0;t<NUM_LEDS-1;t+=2) {
                ledrendered[i*NUM_LEDS+t]=datapacket.rx_outputdata[i].startcolour;
                ledrendered[i*NUM_LEDS+t+1]=datapacket.rx_outputdata[i].endcolour;
              }
              break;
      case 4: // Alternate gradient
              for(t=0;t<NUM_LEDS;t+=2) {
                ledrendered[i*NUM_LEDS+t]=FadeColour(datapacket.rx_outputdata[i].startcolour,datapacket.rx_outputdata[i].endcolour,(t*100)/NUM_LEDS);
              }
              for(t=1;t<NUM_LEDS;t+=2) {
                ledrendered[i*NUM_LEDS+t]=FadeColour(datapacket.rx_outputdata[i].endcolour,datapacket.rx_outputdata[i].startcolour,(t*100)/NUM_LEDS);
              }
              break;
      case 5: // Rainbow
              fill_rainbow(ledrendered+(NUM_LEDS*i),NUM_LEDS,0,1+(datapacket.rx_outputdata[i].patternvaluea/15));
              break;
      case 6: // Solid colour with band running down
              fill_solid(ledrendered+(NUM_LEDS*i),NUM_LEDS,datapacket.rx_outputdata[i].startcolour);
              // x is the centre point for the effect
              // y is the width for the effect (with a minimum of 1)
              x=(int)(((unsigned int)datapacket.rx_outputdata[i].patternvaluea*(unsigned int)(NUM_LEDS-1))/(unsigned int)255);
              y=1+(datapacket.rx_outputdata[i].patternvalueb/25);
                           
              for(t=0;t<y;t++) {
                tcol=FadeColour(datapacket.rx_outputdata[i].startcolour,datapacket.rx_outputdata[i].endcolour,100-((t*100)/y));
                if ((x+t)<NUM_LEDS) ledrendered[i*NUM_LEDS+x+t]=tcol;
                if ((x-t)>0) ledrendered[i*NUM_LEDS+x-t]=tcol;
              }
              // The centre is always full
              ledrendered[i*NUM_LEDS+x]=datapacket.rx_outputdata[i].endcolour;
              break;
      default: // Black
              break;
    }
    // TODO: Perhaps make this independent per chain
    if (datapacket.rx_outputdata[i].glitter>0) {
      // Add some glitter
      if( random8() < datapacket.rx_outputdata[i].glitter) {
        ledrendered[i*NUM_LEDS+random16(NUM_LEDS)] = CRGB::White;
        ledrendered[i*NUM_LEDS+random16(NUM_LEDS)] = CRGB::White;
      }                
      needs_constant_refresh=1;
    }
  }

  // Now copy to the output as required
  #ifdef DICE_MODE
    // Limited to a single string for now
    i=0;
    for(t=1;t<7;t++) {
      for(x=1;x<=t;x++) {
        if (t&0x01) {
          FadeColourInt(ledoutput+i,&BlackLed,ledrendered+i,datapacket.rx_outputmap[0].fader);
        } else {
          FadeColourInt(ledoutput+i,&BlackLed,ledrendered+i+NUM_LEDS,datapacket.rx_outputmap[0].fader);          
        }
        i++;
      }
    }
    FastLED[0].showLeds(BRIGHTNESS);
  #else
    for(i=0;i<NUM_STRINGS;i++) {
      switch (datapacket.rx_outputmap[i].fader) {
        case 0: // Copy straight from ledrendered[0]
                memcpy(ledoutput,ledrendered,NUM_LEDS*3);
                break;
        case 255: // Copy straight from ledrendered[1]
                memcpy(ledoutput,ledrendered+NUM_LEDS,NUM_LEDS*3);
                break;
        default: // Do a multiplication
                for(t=0;t<NUM_LEDS;t++) {
                  FadeColourInt(ledoutput+t,ledrendered+t,ledrendered+t+NUM_LEDS,datapacket.rx_outputmap[i].fader);
                }
                break;
      }
      FastLED[i].showLeds(BRIGHTNESS);
    }
  #endif
}


#ifndef SERIALBRIDGEVERSION

#ifdef ESP32
// Required to build ESP32 version
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif

#ifdef ESP8266
// Required to build ESP8266 version!
void OnDataRecv(unsigned char *mac, unsigned char *incomingData, unsigned char len) {
#endif

  digitalWrite(LED_BUILTIN, HIGH);
  allow_transition_to_ota=0;
  if (len==sizeof(s_controlpacket)) {
    memcpy(&datapacket,incomingData,sizeof(s_controlpacket));
    // refreshoutput();
    refresh_required=1;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

#endif
