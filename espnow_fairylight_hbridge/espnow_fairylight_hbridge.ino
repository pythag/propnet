// Compile using LOLIN(WEMOS) D1 R2 and mini
#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t dmxmode,levela,levelb;
uint8_t a,b;
uint8_t isrstage=0;

const uint16_t D2BIT = 1 << D2;
const uint16_t D3BIT = 1 << D3;

void ICACHE_RAM_ATTR onTimerISR() {
    if (dmxmode<10) {
      switch (isrstage) {
        default:
        case 0: // Reset - everything off using 00 as the output
                if (abs(levela-a)>100) {
                  a=levela;
                } else {
                  if (a>levela) a--;
                  if (a<levela) a++;
                }
                if (abs(levelb-b)>100) {
                  b=levelb;
                } else {
                  if (b>levelb) b--;
                  if (b<levelb) b++;
                }
                GPOC = (D2BIT | D3BIT);
                // The next step is when we want to turn Chain A on
                if (a==255) {
                  // Turn chain A on immediatly
                  GPOS = D2BIT;
                  timer1_write(2560/2);
                  isrstage=2;
                } else {
                  if (a==0) {
                    // Never turn chain A on
                    timer1_write(2560/2);
                    isrstage=2;
                  } else {
                    // Turn chain A on 
                    timer1_write(5*(255-a));
                    isrstage=1;
                  }
                }
                break;
         case 1: // Turn on chain A by setting D2
                GPOS = D2BIT;
                isrstage=2;
                timer1_write(10*a);
                break;
         case 2: // Turn everything off using 11 as the output
                GPOS = (D2BIT | D3BIT);
                // The next step is when we want to turn Chain A on
                if (b==255) {
                  GPOC = D2BIT;
                  timer1_write(2560/2);
                  isrstage=0;
                } else {
                  if (b==0) {
                    timer1_write(2560/2);
                    isrstage=0;
                  } else {
                    timer1_write(5*(255-b));
                    isrstage=3;
                  }
                }
                break;
          case 3: // Turn on chain B by clearing D2
                GPOC = D2BIT;
                isrstage=0;
                timer1_write(10*b);
                break;             
      }
    } else {
      // Strobing mode
      if (isrstage>1) isrstage=0;
      timer1_write(2048*(259-dmxmode));
      if (isrstage==0) {
        // Turn all off
        if ((levela>10)&&(levelb>10)) {
          if (levela>levelb) {
            // Red
            GPOS = D3BIT;
            GPOC = D2BIT;
          } else {
            // Blue
            GPOS = D2BIT;
            GPOC = D3BIT;
          }
        } else {
          // Go back to black
          GPOC = (D2BIT | D3BIT);          
        }
      } else {
        if ((levela>10)||(levelb>10)) {
          if (levela>levelb) {
            // Blue
            GPOC = D3BIT;
            GPOS = D2BIT;
          } else {
            // Red
            GPOC = D2BIT;
            GPOS = D3BIT;
          }
        }
      }
      a=1;
      isrstage++;
    }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  dmxmode=0;
  levela=255;
  levelb=255;
  a=0;
  b=0;

  GPOC = (D2BIT | D3BIT);
  GPOS = D2BIT;
  delay(1000);
  
  GPOC = (D2BIT | D3BIT);
  GPOS = D3BIT;
  delay(1000);

  // Needed to stop a boot loop
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv); 

  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(60); 

}

void loop() {
  delay(1);
}

void OnDataRecv(unsigned char *mac, unsigned char *incomingData, unsigned char len) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (len==21) {
    levela=incomingData[0];
    levelb=incomingData[1];
    dmxmode=incomingData[2];
  }
  digitalWrite(LED_BUILTIN, LOW);
}
