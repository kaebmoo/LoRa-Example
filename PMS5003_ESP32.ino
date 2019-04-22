// On Leonardo/Micro or others with hardware serial, use those!
// uncomment this line:
// #define pmsSerial Serial1

// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial

// For Wemos pin #D5
/*
 * MIT License

  Copyright (c) 2019 Pornthep Nivatyakul
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

 * 
 */

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

 // References:
 // [feather] adafruit-feather-m0-radio-with-lora-module.pdf

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Timer.h>  // https://playground.arduino.cc/Code/Timer/ 
#include <ArduinoJson.h>
#include <CayenneLPP.h>

// milli second
#define ONEDAY 86400000
#define ONEHOUR 3600000
#define ONEMINUTE 60000
#define FIVEMINUTE 300000

#define LED 2

// LoRaWAN Config
#define CFG_as923

// The library converts the address to network byte order as needed.
static const u4_t DEVADDR = 0xXXXXXXXX ; // <-- Change this address for every node!

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX,0xXX };

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[60] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// second
const unsigned TX_INTERVAL = 300;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
// change to TTGO ESP32 LoRa v1
const lmic_pinmap lmic_pins = {
    .nss = 18,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,                       // reset pin
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};


// #include <SoftwareSerial.h>
// SoftwareSerial pmsSerial(D5, D6);

// #include "SoftwareSerialESP32.h"
// SoftwareSerial pmsSerial(33, 34, false, 64); // RX, TX
HardwareSerial pmsSerial(2);

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

uint16_t minimum25, maximum25, minimum10, maximum10;
int pm25[512], pm10[512];
int averagePM25 = 0;
int averagePM10 = 0;
// Declare a buffer to hold the result
char outputData[60], outputMaxMin[60];
CayenneLPP lpp(60);

Timer timerPrepareData;
Timer timerClearData; // 24 hr.
Timer timerSendMaxMin;
int countData = 0;
bool scheduleMaxMin = false;

void setup() 
{
  int onlyOne = 0;

  pinMode(LED, OUTPUT); 
  
  // our debugging output
  Serial.begin(115200);

  // sensor baud rate is 9600
  pmsSerial.begin(9600);

  delay(1000);
  Serial.println();
  Serial.println("Starting...");
  delay(1000);

  loraSetup();

  // clear array for keep data
  for (int index = 0; index < 512; index++) {
    pm25[index] = -1;
    pm10[index] = -1;
  }
  countData = 0;
  // warm sensor 
  for (int i = 20; i > 0; i--) {
    while (!readPMSdata(&pmsSerial)) { 
           
    }
    // reading data was successful!
    pm25[countData] = data.pm25_env;
    pm10[countData] = data.pm10_env;
    countData++;
    if (onlyOne == 0) {
      minimum10 = data.pm10_env;
      maximum10 = data.pm10_env;
      minimum25 = data.pm25_env;
      maximum25 = data.pm25_env;
      onlyOne = 1;
    }
    
    if (data.pm10_env > maximum10) {
      maximum10 = data.pm10_env;
    }
    if (data.pm10_env < minimum10) {
      minimum10 = data.pm10_env;
    }
    if (data.pm25_env > maximum25) {
      maximum25 = data.pm25_env;
    }
    if (data.pm25_env < minimum25) {
      minimum25 = data.pm25_env;
    }    
    Serial.print(i); Serial.print(" ");
  }
  // reading data was successful!
  Serial.println();
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (standard)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
  
  
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
  Serial.println("---------------------------------------");

  Serial.print("PM 2.5 Max:"); Serial.println(maximum25);
  Serial.print("PM 2.5 Min:"); Serial.println(minimum25);
  Serial.println("---------------------------------------");
  

  timerPrepareData.every(FIVEMINUTE, prepareData);
  timerClearData.every(ONEDAY, clearData);
  timerSendMaxMin.every(FIVEMINUTE, sendMaxMin);
  prepareData();
  
  delay(5000);
  // Start job
  do_send(&sendjob);

  Serial.println("loop starting...");
  countData = 0;
}
    
void loop() 
{
  
  unsigned long now;

  now = millis();
  if ((now & 512) != 0) {
    digitalWrite(2, HIGH);
  }
  else {
    digitalWrite(2, LOW);
  }
  
  timerPrepareData.update();
  timerClearData.update();
  timerSendMaxMin.update();  
  os_runloop_once();

  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    pm25[countData] = data.pm25_env;
    pm10[countData] = data.pm10_env;
    countData++;
    if (countData >= 512) {
      countData = 0;
    }
    Serial.print(".");
    /*
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    */
    if (data.pm10_env > maximum10) {
      maximum10 = data.pm10_env;
    }
    if (data.pm10_env < minimum10) {
      minimum10 = data.pm10_env;
    }
    if (data.pm25_env > maximum25) {
      maximum25 = data.pm25_env;
    }
    if (data.pm25_env < minimum25) {
      minimum25 = data.pm25_env;
    }
    /*
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");

    Serial.print("PM 2.5 Max:"); Serial.println(maximum25);
    Serial.print("PM 2.5 Min:"); Serial.println(minimum25);
    Serial.println("---------------------------------------");
    */
  }
}

void sendMaxMin()
{
  scheduleMaxMin = true;
}

void clearData()
{
  // clear max min data;
  minimum10 = data.pm10_env;
  maximum10 = data.pm10_env;
  minimum25 = data.pm25_env;
  maximum25 = data.pm25_env;
}

void cayenneLppData()
{
  lpp.reset();
  lpp.addDigitalInput(1, averagePM25);
  lpp.addDigitalInput(1, averagePM10);
  lpp.addDigitalInput(1, maximum25);
  lpp.addDigitalInput(1, minimum25);
}

void jsonData()
{
  
  // https://arduinojson.org/v5/assistant/
  const size_t capacity = JSON_OBJECT_SIZE(3);
  DynamicJsonBuffer jsonBuffer(capacity);
  
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = "PM5003";
  root["pm25"] = averagePM25;
  root["pm10"] = averagePM10;

  Serial.println();
  root.printTo(Serial);
  Serial.println();
  
  // Produce a minified JSON document
  root.printTo(outputData);
}

void jsonMaxMin()
{
  // https://arduinojson.org/v5/assistant/
  const size_t capacity = JSON_OBJECT_SIZE(3);
  DynamicJsonBuffer jsonBuffer(capacity);
  
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = "PM5003";
  root["pm25max"] = maximum25;
  root["pm25min"] = minimum25;
  
  Serial.println();
  root.printTo(Serial);
  Serial.println();

  // Produce a minified JSON document
  root.printTo(outputMaxMin);
}

void prepareData()
{
  averageData();
  jsonData();
  jsonMaxMin();
  cayenneLppData();
}

void averageData()
{
  int index;
  averagePM25 = 0;
  averagePM10 = 0;
  
  Serial.println();
  Serial.print("Number of data : ");
  Serial.println(countData);
  countData = 0;
  Serial.println("---------------------------------------");
  Serial.print("PM 2.5 Max:"); Serial.println(maximum25);
  Serial.print("PM 2.5 Min:"); Serial.println(minimum25);
  Serial.println("---------------------------------------");

  for (index = 0; pm25[index] != -1; index++) {
    averagePM25 += pm25[index];
  }
  if (index > 0) {
    averagePM25 = averagePM25 / index;
  }
  Serial.print("PM 2.5 Average:"); Serial.println(averagePM25);

  for (index = 0; pm10[index] != -1; index++) {
    averagePM10 += pm10[index];
  }
  if (index > 0) {
    averagePM10 = averagePM10 / index;
  }
  Serial.print("PM 10 Average:"); Serial.println(averagePM10);

  // clear array for keep data
  for (index = 0; index < 512; index++) {
    pm25[index] = -1;
    pm10[index] = -1;
  }  
  
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

// LoRaWAN
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        // case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
  int i;
  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
        // Prepare upstream data transmission at the next possible time.
        // LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.print("my data: ");
        Serial.println(outputData);

        for (i = 0; outputData[i] != '\0'; i++) {
          mydata[i] = outputData[i];          // copy json data format
        }
        
        // LMIC_setTxData2(1, mydata, i, 0);   // send data in json format

        // send data in CayenneLPP 
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        
        Serial.println(F("Packet queued"));
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);

        /*
        if(scheduleMaxMin) {
          delay(2000);
          Serial.print("my max/min data: ");
          Serial.println(outputMaxMin);
          
          for (i = 0; outputMaxMin[i] != '\0'; i++) {
            mydata[i] = outputMaxMin[i];
          }
          LMIC_setTxData2(1, mydata, i, 0);

          Serial.println(F("Packet queued"));
          Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
          
          scheduleMaxMin = false;
        }
        */
         
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void loraSetup()
{
      // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    // LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    // LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
    
    
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    
    // LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    
    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 924800000, DR_RANGE_MAP(DR_FSK, DR_FSK),   BAND_MILLI);      // g2-band
    #endif

    // disable channel 2 - channel 8.
    for (int i = 2; i < 9; i++) {
      LMIC_disableChannel(i);
    }

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    // LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // LMIC_setDrTxpow(DR_SF7,14);
    LMIC_setDrTxpow(DR_SF9,20);

    // Start job
    // do_send(&sendjob);
}
