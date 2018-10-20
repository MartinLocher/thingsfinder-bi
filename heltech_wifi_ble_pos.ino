#include <Arduino.h>
#define CFG_sx1276_radio 1
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
#include <TinyGPS.h>

// OLED Pins
#define OLED_SCL 15   // GPIO 15h
#define OLED_SDA  4   // GPIO  4
#define OLED_RST 16   // GPIO 16

// LoRa Pins
#define LoRa_RST  14  // GPIO 14
#define LoRa_CS   18  // GPIO 18
#define LoRa_DIO0 26  // GPIO 26

#define HELTECV1 & TTBEAM
#ifdef HELTECV1
#define LoRa_DIO1 33  // GPIO 33
#define LoRa_DIO2 32  // GPIO 32
#endif

//#define HELTECV2
#ifdef HELTECV2
#define LoRa_DIO1 34  // GPIO 33
#define LoRa_DIO2 35  // GPIO 32
#endif
// define the display type that we use
#ifdef LCD_DISP
static U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);
#endif

TinyGPS gps;

RTC_DATA_ATTR boolean send_always = true;
RTC_DATA_ATTR byte sf = 10;
RTC_DATA_ATTR int seqno_up = 0;
RTC_DATA_ATTR boolean first_run = true;

#ifdef TTGO_BUG
RTC_DATA_ATTR boolean in_sleep = false;
RTC_DATA_ATTR int wake_cnt = 0;
#endif

#define LORA_MSG_PORT 66
#define NO_STARTUP_BLINKS 5
#define STARTUP_BLINK_PERIOD  300
#define SLEEP_MODE

//#define BIBERNODE1
#ifdef BIBERNODE1
static const PROGMEM u1_t NWKSKEY[16] = { 0x2D, 0x89, 0xF5, 0x50, 0x64, 0x06, 0x3B, 0xA3, 0x67, 0xB8, 0x71, 0x80, 0x63, 0x6C, 0xB2, 0xA1 };
static const u1_t PROGMEM APPSKEY[16] = { 0x73, 0x8A, 0xD7, 0xD0, 0x17, 0x67, 0x5D, 0xF2, 0xC2, 0x11, 0xC4, 0x71, 0x3A, 0x97, 0x4D, 0x7E };
static const u4_t DEVADDR = 0x260118EC ;
#endif


//#define BIBERNODE2
#ifdef BIBERNODE2
static const u1_t NWKSKEY[16] = { 0xA2, 0x90, 0xE6, 0x58, 0xD0, 0x5A, 0x1E, 0x1B, 0x99, 0x84, 0x6C, 0xD0, 0xD1, 0x97, 0xFD, 0x1C };
static const u1_t APPSKEY[16] = { 0x85, 0xB1, 0x18, 0x2A, 0xA1, 0x90, 0x82, 0xD4, 0xD5, 0xDA, 0x3F, 0x48, 0xF3, 0x6C, 0xCF, 0x56 };
static const u4_t DEVADDR = 0x260115AE;
#endif

//#define BIBERNODE3
#ifdef BIBERNODE3
static const u1_t NWKSKEY[16] = { 0xBC, 0xCC, 0xDC, 0xB1, 0x65, 0x7A, 0x04, 0x90, 0x44, 0x42, 0x14, 0x6D, 0x47, 0xA1, 0xE5, 0xB6 };
static const u1_t APPSKEY[16] = { 0x56, 0x44, 0x94, 0xDC, 0x86, 0xEB, 0xB9, 0xF9, 0x2C, 0xF5, 0x69, 0xF3, 0x42, 0xA9, 0x5B, 0x13 };
static const u4_t DEVADDR = 0x26011F73;
#endif

#define BIBERNODE4
#ifdef BIBERNODE4
static const u1_t NWKSKEY[16] = { 0x3B, 0xD4, 0xF6, 0xC7, 0x2D, 0x09, 0xBD, 0x5C, 0x4C, 0xAE, 0x79, 0x23, 0x01, 0x47, 0x21, 0xDB };
static const u1_t APPSKEY[16] = { 0x51, 0x5C, 0xB8, 0xE4, 0x75, 0x2D, 0x55, 0xC2, 0xF4, 0x93, 0x84, 0xDA, 0x86, 0x98, 0xEB, 0xF1 };
static const u4_t DEVADDR = 0x26011CE4;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#ifdef BIBERNODE1
RTC_DATA_ATTR byte dev_unique_id = 0x20;
#endif


#ifdef BIBERNODE2
RTC_DATA_ATTR byte dev_unique_id = 0x21;
#endif

#ifdef BIBERNODE3
RTC_DATA_ATTR byte dev_unique_id = 0x22;
#endif

#ifdef BIBERNODE4
RTC_DATA_ATTR byte dev_unique_id = 0x23;
#endif

RTC_DATA_ATTR byte room_number = 0x00;

#define MAX_MAC 2 // number of MAC addresses to sent
static char mydata[1 + MAX_MAC * 7 + MAX_MAC] ; //*Space for Mode + (2 MACs + 2 RSSI9 + Unique ID + Room ID
String mydata_str;
static osjob_t sendjob;


#define LED_OFF   0
#define LED_ON    LED_OFF  + 1

#define LED_PIN    25

RTC_DATA_ATTR byte LED_STATE = LED_OFF;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

#define ONE_MINUTE 60
//#define ONE_MINUTE 1
RTC_DATA_ATTR unsigned TX_INTERVAL = 1; //in minutes
#define uS_TO_S_FACTOR 1000000
// Schedule TX every this many minutes (might become longer due to duty
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
#define BLE_POS  WIFI_POS + 1
#define GPS_POS  BLE_POS + 1

RTC_DATA_ATTR int mode = BLE_POS;


#define LCD_OFF  0
#define LCD_ON  LCD_OFF + 1

RTC_DATA_ATTR int show_lcd_msg = LCD_ON;


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

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



void set_led (byte led_stat)
{
  Serial.print("LED set to: ");
  Serial.println(led_stat);
  LED_STATE = led_stat;
  digitalWrite (LED_PIN, LED_STATE);
}

void set_lcd (byte lcd_stat)
{

#ifdef  LCD_DISP
  Serial.print("LCD set to: ");
  Serial.println(lcd_stat);
  show_lcd_msg = lcd_stat;

  if (show_lcd_msg == LCD_OFF)
  {

    u8x8.setPowerSave(1);
  }
  else
  {

    u8x8.setPowerSave(0);
    /*
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);

      u8x8.drawString(0, 1, "BI THINGSFinder");

      u8x8.drawString(0, 5, "Ready");
    */
  }
#endif
}

void  set_transmit_SF(byte strength)
{
  Serial.print("SF set to: ");
  Serial.println(strength);
  switch (strength)
  {
    case 7:
      sf = 7;
      LMIC_setDrTxpow(DR_SF7, 14);
      break;
    case 8:
      sf = 8;
      LMIC_setDrTxpow(DR_SF8, 14);
      break;
    case 9:
      sf = 9;
      LMIC_setDrTxpow(DR_SF9, 14);
      break;
    case 10:
      sf = 10;
      LMIC_setDrTxpow(DR_SF10, 14);
      break;
    case 11:
      sf = 11;
      LMIC_setDrTxpow(DR_SF11, 14);
      break;
    case 12:
      sf = 12;
      LMIC_setDrTxpow(DR_SF12, 14);
      break;
  }
}


void set_mode (byte whish_mode)
{
  Serial.print("Mode set to: ");
  Serial.println(mode);

  mode = whish_mode;
}

void  set_sleep(byte sleeptime)
{
  Serial.print("Sleeptime set to: ");
  Serial.println(sleeptime);

  TX_INTERVAL = sleeptime;
}

void set_sendallways(byte flag)
{
  Serial.print("Sendallways set to: ");
  Serial.println(flag);

  if (flag > 0)
    send_always = true;
  else
    send_always = false;
}

void set_unique_id( byte id )
{
  Serial.print("Unique_Id set to: ");
  Serial.println(id);
  dev_unique_id = id;
}

void set_room_number( byte room )
{
  Serial.print("Room set to: ");
  Serial.println(room);
  room_number = room;
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
        Serial.println();

        // on/off cmds should be coded bitwise in just one byte not by using a complete array  !!!!
        // future release
        switch (LMIC.dataLen)
        {

          case 1:
            set_led (LMIC.frame[LMIC.dataBeg]);
            break;


          case 2:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg] + 1);

            break;

          case 3:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);

            break;

          case 4:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);
            set_transmit_SF(LMIC.frame[LMIC.dataBeg + 3]);

            break;

          case 5:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);
            set_transmit_SF(LMIC.frame[LMIC.dataBeg + 3]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 4]);


            break;

          case 6:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);
            set_transmit_SF(LMIC.frame[LMIC.dataBeg + 3]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 4]);
            set_sendallways(LMIC.frame[LMIC.dataBeg + 5]);

            break;

          case 7:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);
            set_transmit_SF(LMIC.frame[LMIC.dataBeg + 3]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 4]);
            set_sendallways(LMIC.frame[LMIC.dataBeg + 5]);
            set_unique_id(LMIC.frame[LMIC.dataBeg + 6]);

            break;

          case 8:
            set_led (LMIC.frame[LMIC.dataBeg]);
            set_lcd (LMIC.frame[LMIC.dataBeg + 1]);
            set_mode(LMIC.frame[LMIC.dataBeg + 2]);
            set_transmit_SF(LMIC.frame[LMIC.dataBeg + 3]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 4]);
            set_sendallways(LMIC.frame[LMIC.dataBeg + 5]);
            set_unique_id(LMIC.frame[LMIC.dataBeg + 6]);
            set_room_number(LMIC.frame[LMIC.dataBeg + 7]);

            // 00 00 00 07 0A 00 0A 40 : led_off, no lcd, wifi, sf7, 10 minutes sleep, not all send, uid=10, room=64
            // 00 00 01 07 01 00 0A 40 : led_off, no_lcd, BLE, sf7, 1 minute sleep, not all send, uid 01, room=64
            /*
              00 00 01 07 00 00 0A 00 (SF7), LED OFF, no sleep
              00 00 01 0A 00 00 0A 00 (SF10) LED OFF, no sleep
              01 00 01 0A 00 00 0A 00 (SF10) LED ON, no sleep
              01 00 01 0A 0A 00 0A 00 LED ON, LCD OFF, BLE MODE, SF, Sleeptime 0, UniqueID, Room                                     
            */
            break;

        }

      }
      Serial.println();

      // Schedule next transmission

#ifdef SLEEP_MODE
      seqno_up ++;
      esp_sleep_enable_timer_wakeup(TX_INTERVAL * ONE_MINUTE * uS_TO_S_FACTOR);
#ifdef TTGO_BUG
      in_sleep = true;
      wake_cnt = 0;
#endif
      esp_deep_sleep_start();
#else
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL * ONE_MINUTE), do_send);
#endif
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
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {

    int l = 0;
    int initial_mode = mode;

    if ( mode == BLE_POS )
    {
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "Scan BLEs ");
      }
#endif
      l = do_ble_scanAndsort();

      if ( l == 0 )
      {
        mode = WIFI_POS;
      }
    }

    if ( mode == WIFI_POS )
    {
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "Scan Wifis");
      }
#endif
      l = do_wifi_scanAndSort();

      if ( l == 0 )
      {
        mode = GPS_POS;
      }
    }

    if ( mode == GPS_POS )
    {
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "Scan GPS");
      }
#endif
      l = do_gps_scan();
    }

    mode = initial_mode;

    if (l > 0)
    {

#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "Send Data");
      }
#endif
      uint8_t * macptr;

      macptr = (uint8_t * )mydata;
      for (int i = 0; i < sizeof(mydata); i ++)
        Serial.printf("%02x", *(macptr + i));
      LMIC_setTxData2(LORA_MSG_PORT, (xref2u1_t)mydata, sizeof(mydata), 0);

      Serial.println(F("Packet queued"));
    }
    else
    {
      if (send_always)
        LMIC_setTxData2(66, (unsigned char *)"hello", sizeof("hello"), 0); //Sent packet to TTN
      // Schedule next transmission
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "NO Device...");
      }
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
  int act_cnt = 0;

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
    mydata[0] = mode;

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
          mydata[1 + j + 7 * act_cnt] = *macptr++;
        } //Mac, 0a:0b:cf:d8:b0:c0: getnative(): 0a0bcfd8b0c0,
        Serial.println();

        mydata[1 + 6 + 7 * act_cnt] = foundDevices.getDevice(indices[i]).getRSSI();
        act_cnt ++;


        /*mac 0:
           0a0bcfd8b0c0
           mac1:
           0b0bcfd8b0c0
           for mac0 mydata:0a0bcfd8b0c00a
        */
      }
      //mydata:0a0bcfd8b0c00a0b0bcfd8b0c00b

      *(mydata + MAX_MAC * 7 + 1) = dev_unique_id;  //1 + mydata +  2 * 7
      *(mydata + MAX_MAC * 7 + 2) = room_number;    //1 + mydata +  2 * 7 + 1
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "  SEND");
      }
#endif
    }

  }
  return act_cnt;
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
    int i;
    memset(mydata, 0, sizeof(mydata));

    mydata[0] = mode;
    for (i = 0; i < l; i++)
    {
      Serial.println(i);
      macptr = WiFi.BSSID(indices[i]);
      for (int j = 0; j < 6; j++) {
        Serial.printf("%02x", *macptr);
        if (j < 5)
        {
          Serial.print(':');
        }
        mydata[1 + j  + 7 * i] = *macptr++;
      }
      Serial.print(" RSSI: ");
      mydata[1 + 6 + 7 * i] = WiFi.RSSI(indices[i]);
      Serial.print(WiFi.RSSI(indices[i]));
      Serial.println();
#ifdef LCD_DISP
      if (show_lcd_msg)
      {
        u8x8.drawString(0, 3, "  SEND");
      }
#endif
    }

    *(mydata + i * 7 + 1) = dev_unique_id;
    *(mydata + i * 7 + 2) = room_number;

  }
  return (n);
}

int do_gps_scan() {

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat, flon, faltitudeGPS, fhdopGPS;
  unsigned long age;

  uint8_t * macptr;
  Serial.println("Getting GPS position...");

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }

  if ( newData ) {
    gps.f_get_position(&flat, &flon, &age);
    faltitudeGPS = gps.f_altitude();
    fhdopGPS = gps.hdop();
    flat = (flat == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flat;
    flon = (flon == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flon;
  }

  gps.stats(&chars, &sentences, &failed);

  //flat = 48.113980;
  //flon = 9.803513;

  int32_t lat = flat * 10000;
  int32_t lon = flon * 10000;
  //int16_t altitudeGPS = faltitudeGPS * 100;
  //int8_t hdopGPS = fhdopGPS;

  // Pad 2 int32_t to 6 8uint_t, big endian (24 bit each, having 11 meter precision)
  uint8_t coords[6];

  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;

  //coords[6] = altitudeGPS;
  //coords[7] = altitudeGPS >> 8;

  //coords[8] = hdopGPS;

  memset(mydata, 0, sizeof(mydata));
  mydata[0] = mode;

  Serial.println(lat);
  Serial.println(lon);
  Serial.println("GPS coords:");
  for ( int j = 0; j < 6; j++) {

    Serial.print(coords[j]);
    mydata[j + 1] = coords[j];

  }
  mydata[7] = dev_unique_id;
  mydata[8] = room_number;

  Serial.println();

  return 1;
}

void do_blink()
{
  digitalWrite (LED_PIN, LED_ON);
  delay (STARTUP_BLINK_PERIOD);
  digitalWrite (LED_PIN, LED_OFF);
  delay (STARTUP_BLINK_PERIOD);
}


void setup() {
  // init packet counter
  //sprintf(mydata, "Packet = %5u", packetNumber);


  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
  Serial.println(F("Starting"));
  pinMode(LED_PIN, OUTPUT);

  // Added 16.10.2018 ML
  Serial1.begin(9600, SERIAL_8N1, 12, 15);

  if (first_run)
  {
    first_run = false;

    for (int blink = 0; blink < NO_STARTUP_BLINKS; blink ++)
      do_blink();

  }



  delay(1000); //Take some time to open up the Serial Monitor
  Serial.println(F("Starting"));
  pinMode(LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LED_STATE);
#ifdef TTGO_BUG
  Serial.println(in_sleep);
  Serial.println(wake_cnt);


  if (in_sleep == true)
  {
    wake_cnt ++;
    if (wake_cnt > 10)
    {
      wake_cnt = 0;
      in_sleep = false;
    }
    else
    {
      esp_sleep_enable_timer_wakeup(TX_INTERVAL * ONE_MINUTE * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  }
#endif
  // set up the display
#ifdef LCD_DISP
  if (show_lcd_msg)
  {
    u8x8.begin();
    u8x8.setPowerSave(0);
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    u8x8.drawString(0, 1, "BI THINGSFinder");

    u8x8.drawString(0, 5, "Ready");
  }
#endif

  /*

    BLEDevice::init("BI_ThingsFinder");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE
                                         );

    pCharacteristic->setValue("MADDDIN says yes");
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  */
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
#ifdef SLEEP_MODE
  LMIC.seqnoUp = seqno_up;
#endif
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  set_transmit_SF (sf);
  //LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
