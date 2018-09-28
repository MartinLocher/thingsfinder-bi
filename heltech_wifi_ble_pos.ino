#include <Arduino.h>

// OLED
#define LCD_DISP
#ifdef LCD_DISP
#include <U8x8lib.h>
#endif

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <WiFi.h>
// OLED Pins
#define OLED_SCL 15   // GPIO 15h
#define OLED_SDA  4   // GPIO  4
#define OLED_RST 16   // GPIO 16

// LoRa Pins
#define LoRa_RST  14  // GPIO 14
#define LoRa_CS   18  // GPIO 18
#define LoRa_DIO0 26  // GPIO 26
#define LoRa_DIO1 33  // GPIO 33
#define LoRa_DIO2 32  // GPIO 32

// define the display type that we use
#ifdef LCD_DISP
static U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);
#endif

/*

#define SERIAL_BAUD       57600

#define WIFI_DELAY        500


#define MAX_CONNECT_TIME  30000
*/

boolean send_always = true;

#define bibernode2
#ifdef bibernode2
static const u1_t NWKSKEY[16] = { 0xA2, 0x90, 0xE6, 0x58, 0xD0, 0x5A, 0x1E, 0x1B, 0x99, 0x84, 0x6C, 0xD0, 0xD1, 0x97, 0xFD, 0x1C };

static const u1_t APPSKEY[16] = { 0x85, 0xB1, 0x18, 0x2A, 0xA1, 0x90, 0x82, 0xD4, 0xD5, 0xDA, 0x3F, 0x48, 0xF3, 0x6C, 0xCF, 0x56 };

static const u4_t DEVADDR = 0x260115AE;
#endif
//#define BIBERNODE1
#ifdef BIBERNODE1
static const PROGMEM u1_t NWKSKEY[16] = { 0x2D, 0x89, 0xF5, 0x50, 0x64, 0x06, 0x3B, 0xA3, 0x67, 0xB8, 0x71, 0x80, 0x63, 0x6C, 0xB2, 0xA1 };
static const u1_t PROGMEM APPSKEY[16] = { 0x73, 0x8A, 0xD7, 0xD0, 0x17, 0x67, 0x5D, 0xF2, 0xC2, 0x11, 0xC4, 0x71, 0x3A, 0x97, 0x4D, 0x7E };
static const u4_t DEVADDR = 0x260118EC ;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }



#define MAX_MAC 2 // number of MAC addresses to sent
static char mydata[MAX_MAC * 7] ;
String mydata_str;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL = 6;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = LoRa_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LoRa_RST,
  .dio = { LoRa_DIO0, LoRa_DIO1, LoRa_DIO2 },
};

const char *DEVICE_NAME = "Gigaset G-tag";
//const char * SERVICE_DATA_UUID = "0000feaa-0000-1000-8000-00805f9b34fb";
//must be executed in root
//hciconfig hci0 up 
//hciconfig hci0 leadv 3
//hcitool -i hci0 cmd 0x08 0x0008 1a 02 01 06 03 03 aa fe 12 16 aa fe 10 00 01 72 61 73 70 62 65 72 72 79 70 69 01 00 00 00 00 00
const int scanTime = 30; 

#define WIFI_POS  0
#define  BLE_POS  WIFI_POS + 1

int mode = BLE_POS;


class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      Serial.print("RSSI: ");
      Serial.println(advertisedDevice.getRSSI());
    }
};


void  set_transmit(byte strength)
{
  Serial.print("strength: ");
  Serial.println(strength);
  switch (strength)
  {
    case 7:
      LMIC_setDrTxpow(DR_SF7, 14);
      break;
    case 8:
      LMIC_setDrTxpow(DR_SF8, 14);
      break;
    case 9:
      LMIC_setDrTxpow(DR_SF9, 14);
      break;
    case 10:
      LMIC_setDrTxpow(DR_SF10, 14);
      break;
    case 11:
      LMIC_setDrTxpow(DR_SF11, 14);
      break;
    case 12:
      LMIC_setDrTxpow(DR_SF12, 14);
      break;
  }
}


void set_mode (byte whish_mode)
{
  Serial.print("Mode: ");
  Serial.println(mode);

  mode = whish_mode;
}

void  set_sleep(byte sleeptime)
{
  Serial.print("Sleeptime: ");
  Serial.println(sleeptime);

  TX_INTERVAL = sleeptime;
}

void set_sendallways(byte flag)
{
  Serial.print("Sendallways: ");
  Serial.println(flag);

  if (flag > 0)
    send_always = true;
  else
    send_always = false;
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    
    int l = 0;

    if (mode == BLE_POS)
    {
      #ifdef LCD_DISP
      u8x8.drawString(0, 2, "Scan BLEs ");
      #endif
      l = do_ble_scanAndsort();
    }
    else
    {
      if (mode == WIFI_POS)
      {
        #ifdef LCD_DISP
            u8x8.drawString(0, 2, "Scan Wifis");
            #endif
       l = do_wifi_scanAndSort();
      }
    }


    if (l > 0)
    {

      #ifdef LCD_DISP
      u8x8.drawString(0, 2, "Send Data");
      #endif

      LMIC_setTxData2(66, (xref2u1_t)mydata, MAX_MAC * 7, 0);

      Serial.println(F("Packet queued"));
    }
    else
    {
      if (send_always)
        LMIC_setTxData2(66, (unsigned char *)"hello", sizeof("hello"), 0); //Sent packet to TTN
      // Schedule next transmission
      #ifdef LCD_DISP
      u8x8.drawString(0, 2, "NO Device...");
      #endif
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

int do_ble_scanAndsort()
{
  //Afegit per tractar cada dispositiu
  BLEAdvertisedDevice device;
  uint8_t * macptr;
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  BLEScanResults foundDevices = pBLEScan->start(scanTime);

  int n = foundDevices.getCount();
  int real_devs = 0;
  Serial.println("list of bles");
  for (int i = 0; i < n; i++)
  {

    Serial.println (foundDevices.getDevice(i).getName().c_str());
    if (foundDevices.getDevice(i).getName() == DEVICE_NAME)
      real_devs ++;
    
    //Serial.println (foundDevices.getDevice(i).getServiceDataUUID().toString().c_str() );
    /*
    if ( foundDevices.getDevice(i).getServiceDataUUID().toString() == SERVICE_DATA_UUID )
    {
      real_devs ++;
    }
    */
  }


  if (real_devs == 0)
  {
    Serial.println("None of the Spec BLEs found!");
  }
  else
  {
    Serial.print("Devices found: ");
    Serial.println(real_devs);
    Serial.println("Scan done!");

    // Create indexes for sorting
    int indices[n];
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }

    // Bubble sort
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if ((foundDevices.getDevice(indices[j]).getRSSI()) > (foundDevices.getDevice(indices[i]).getRSSI()))
          std::swap(indices[i], indices[j]);
      }
    }

    memset(mydata, 0, sizeof(mydata));
    int act_cnt = 0;
    for (int i = 0; i < n; i++)
    {
      if ((foundDevices.getDevice(indices[i]).getName() == DEVICE_NAME) && (act_cnt < MAX_MAC))
      
      //if ((foundDevices.getDevice(i).getServiceDataUUID().toString() == SERVICE_DATA_UUID) && (act_cnt < MAX_MAC))
      {
        Serial.println("****");
        Serial.println(foundDevices.getDevice(indices[i]).getName().c_str());
        Serial.print(" RSSI: ");
        Serial.println(foundDevices.getDevice(indices[i]).getRSSI());
        Serial.print (" Device Mac: ");
        Serial.println(foundDevices.getDevice(indices[i]).getAddress().toString().c_str());

        macptr = (uint8_t *)(foundDevices.getDevice(indices[i]).getAddress().getNative());

        for (int j = 0; j < 6; j++) {
          Serial.printf("%02x", *macptr);
          if (j < 5)
          {
            Serial.print(':');
          }
          mydata[j + 7 * act_cnt] = *macptr++;
        }
        Serial.println();

        mydata[6 + 7 * act_cnt] = foundDevices.getDevice(indices[i]).getRSSI();
        act_cnt ++;
      }
      #ifdef LCD_DISP
      u8x8.drawString(0, 2, "SSEND");
      #endif
    }

  }
  return real_devs;
}
/*
  /* Scan available networks and sort them in order to their signal strength. */

int do_wifi_scanAndSort() {

  uint8_t * macptr;
  int n = WiFi.scanNetworks();
  Serial.println("Scan done, sort starting!");
  if (n == 0) {
    Serial.println("No networks found!");
  } else {
    Serial.print(n);
    Serial.println(" networks found.");
    int indices[n];
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
          std::swap(indices[i], indices[j]);
        }
      }
    }

    for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(indices[i]));
      Serial.print(" ");
      Serial.print(WiFi.RSSI(indices[i]));
      Serial.print(" ");
      Serial.print(WiFi.BSSIDstr(indices[i]));
      Serial.print(" ");
      Serial.print(WiFi.encryptionType(indices[i]));
      Serial.println();
    }
    Serial.println("Best BSSID: ");
    int l;
    if (n > MAX_MAC)
      l = MAX_MAC;
    else
      l = n;

    for (int i = 0; i < l; i++)
    {
      Serial.println(i);
      macptr = WiFi.BSSID(indices[i]);
      for (int j = 0; j < 6; j++) {
        Serial.printf("%02x", *macptr);
        if (j < 5)
        {
          Serial.print(':');
        }
        mydata[j + 7 * i] = *macptr++;
      }
      Serial.print(" RSSI: ");
      mydata[6 + 7 * i] = WiFi.RSSI(indices[i]);
      Serial.print(WiFi.RSSI(indices[i]));
      Serial.println();
      #ifdef LCD_DISP
      u8x8.drawString(0, 2, "SSEND");
      #endif
    }

  }
  return (n);
}


void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
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
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
        }
        switch (LMIC.dataLen)
        {
          case 1:
            set_mode(LMIC.frame[LMIC.dataBeg]);
            break;

          case 2:
            set_mode(LMIC.frame[LMIC.dataBeg]);
            set_transmit(LMIC.frame[LMIC.dataBeg + 1]);
            break;

          case 3:
            set_mode(LMIC.frame[LMIC.dataBeg]);
            set_transmit(LMIC.frame[LMIC.dataBeg + 1]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 2]);
            break;

          case 4:
            set_mode(LMIC.frame[LMIC.dataBeg]);
            set_transmit(LMIC.frame[LMIC.dataBeg + 1]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 2]);
            set_sendallways(LMIC.frame[LMIC.dataBeg + 3]);
            break;

          case 5:
            set_mode(LMIC.frame[LMIC.dataBeg]);
            set_transmit(LMIC.frame[LMIC.dataBeg + 1]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 2]);
            set_sendallways(LMIC.frame[LMIC.dataBeg + 3]);

            break;

        }

      }
      Serial.println();

      // Schedule next transmission
      // digitalWrite(LED_BUILTIN, HIGH);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}


void setup() {
  // init packet counter
  //sprintf(mydata, "Packet = %5u", packetNumber);

  Serial.begin(115200);
  Serial.println(F("Starting"));

  // set up the display
  #ifdef LCD_DISP
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
  u8x8.drawString(0, 0, "BI THINGSFinder");

  u8x8.drawString(0, 4, "Ready");
  #endif
 
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

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

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // disable channels (only use channel 0 - 868.1 MHz - for my single channel gateway!!!)
  //    for (int channel = 1; channel <= 8; channel++) {
  //      LMIC_disableChannel(channel);
  //    }
  //
  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
