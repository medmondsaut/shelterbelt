#include <SPI.h>
#include <Wire.h>

// LoRaWAN libraries
#include <lmic.h>
#include <hal/hal.h>


// OLED display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// permanent storage for sequence number
#include <FlashStorage.h>
FlashStorage(flash_top, uint16_t);

// gps
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog; //Create instance
const byte OpenLogAddress = 42; //Default Qwiic OpenLog I2C address


// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = {0x9D, 0x24, 0xAB, 0xEB, 0xD2, 0xBB, 0x88, 0x6F, 0xDE, 0x20, 0xCB, 0x11, 0xD1, 0xCB, 0x49, 0x93} ;;  

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0x80, 0x7E, 0x29, 0x5C, 0x31, 0x28, 0x27, 0x5C, 0x6E, 0x8D, 0x4C, 0x30, 0xB7, 0x12, 0x08, 0x3C};

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR =  0x00e7b594 ; // <-- Change this address for every node! For example, our device address is 26022DEN. We will need to replace "DEVICE_ADDRESS_HERE" as 0x26022DEB.

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
// Well alright......
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 12,//RFM Chip Select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,//RFM Reset
    .dio = {6, 10, 11}, //RFM Interrupt, RFM LoRa pin, RFM LoRa pin
};

// housekeeping stuff
unsigned long sequence ;
unsigned long time_val, time_next;
unsigned long time_between = 5000;
static uint8_t payload[5];


void onEvent (ev_t ev) {
    SerialUSB.print(os_getTime());
    SerialUSB.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            SerialUSB.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            SerialUSB.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            SerialUSB.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            SerialUSB.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            SerialUSB.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            SerialUSB.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            SerialUSB.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            SerialUSB.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            SerialUSB.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            SerialUSB.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              SerialUSB.println(F("Received ack"));
            if (LMIC.dataLen) {
              SerialUSB.println(F("Received "));
              SerialUSB.println(LMIC.dataLen);
              SerialUSB.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            SerialUSB.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            SerialUSB.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            SerialUSB.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            SerialUSB.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            SerialUSB.println(F("EV_LINK_ALIVE"));
            break;
         default:
            SerialUSB.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        SerialUSB.println(F("OP_TXRXPEND, not sending"));
    }
    else{

        sequence = sequence + 1 ;
        SerialUSB.println(sequence) ;
        
        payload[4] = (byte) sequence ;
        payload[3] = (byte) (sequence >> 8) ;
        payload[2] = (byte) (sequence >> 16);
        payload[1] = (byte) (sequence >> 24);
        payload[0] = (byte) 0 ;

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        SerialUSB.println(F("Packet queued"));

        String encoded = String(payload[4],HEX);

        long latitude = myGPS.getLatitude();
        long longitude = myGPS.getLongitude();
        byte siv = myGPS.getSIV();

        uint16_t gpsYear = myGPS.getYear();
        uint16_t gpsMonth = myGPS.getMonth();
        uint16_t gpsDay = myGPS.getDay();
        uint16_t gpsHour = myGPS.getHour();
        uint16_t gpsMinute = myGPS.getMinute();
        uint16_t gpsSecond = myGPS.getSecond();

        String logline = 
            String(sequence) + "," + 
            String(siv) + "," + 
            String(latitude) + "," + 
            String(longitude);

        String logtime = 
            String(gpsYear) + "," + 
            String(gpsMonth) + "," + 
            String(gpsDay) + "," + 
            String(gpsHour) + "," + 
            String(gpsMinute) + "," + 
            String(gpsSecond) ; 
            
        Serial.println(logline) ;

        display.setTextSize(1);             // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);        // Draw white text  
        display.clearDisplay(); // Clear display buffer
        display.setCursor(0,0);             // Start at top-left corner
        display.print("Seq:") ;
        display.println(sequence);
        display.print("SIV:") ;
        display.println(siv) ;
        display.println(logtime) ;
        display.println(String(latitude) + "," + String(longitude));
        display.display() ;

        myLog.println(logline + "," + logtime) ;
        
    // Next TX is scheduled after TX_COMPLETE event.
    }
}

void setup() {
    uint16_t topseq = flash_top.read();
    topseq = topseq + 1 ;
    flash_top.write(topseq) ;

    sequence = topseq * 65536 + 1 ;

    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
    }
    display.display();

    SerialUSB.begin(115200);
    // Serial communication on startup is not consistent on the SAMD21.
    delay(2000); 
    SerialUSB.println("Starting");

    Wire.begin();
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
      {
        SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
      }
    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR

    long latitude = myGPS.getLatitude();
    SerialUSB.print(F("Lat: "));
    SerialUSB.print(latitude);

    myLog.begin(); //Open connection to OpenLog (no pun intended)
    myLog.append("logfile.txt");


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
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);


}

void loop() {
    os_runloop_once();
    

}
