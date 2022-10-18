/*
  Galaxy Vehicle Security Module

  https://github.com/cosmicc/GVSM
  Created by: ianperry99@gmail.com
*/

#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <SoftwareSerial.h>
#include <Sim800L.h>
#include <DS3231.h>
#include <uEEPROMLib.h>
#include <TinyGPSPlus.h>
#include <uptime_formatter.h>
#include <Timezone.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ************* PIN DEFINITIONS ***************

#define MODEM_RX_PIN 26            // Sim800 UART RX pin
#define MODEM_TX_PIN 27            // Sim800 UART TX pin
#define MODEM_RESET_PIN 5          // Sim800 Reset Pin
#define MODEM_POWER_PIN 23         // Sim800 Power on Pin
#define GPS_POWER_PIN 4            // GPS Module Power Pin
#define GPS_RX_PIN 33              // GPS UART RX PIN
#define GPS_TX_PIN 32              // GPS UART TX PIN
#define RELAY_PIN 13               // Relay trigger pin
#define SENSOR_PIN 34              // Voltage sensor pin
#define WAKEUP_PIN 27              // Deep sleep external wakeup pin

// ************* OTHER DEFINITIONS ***************

#define GPS_BAUD 9600              // GPS UART gpsSpeed
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define EEPROM_SIZE 2              // ESP32 EEPROM save usage bytes
#define INT_SAMPLES 10             // Internal battery voltage samples
#define EXT_SAMPLES 10             // External battery voltage samples
#define WDT_TIMEOUT 10             // Hardware watchdog timeout (seconds)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID for BLE
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// *************** ENUMS ******************

enum PowerState {OFF, BATTERY, ALTERNATOR, INIT};

// ************* CONSTANTS ***************

const float FACTOR = 5.5;          // reduction factor of the External Voltage Sensor
const bool cellactive = false;     // Activate cell service
const bool RESPOND = false;        // Respond to SMS
const int UNDERTEMP = -20;         // Undertemp Shutdown temperature in Celsius
const int OVERTEMP = 60;           // Overtemp Shutdown temperature in Celsius

// ************* GLOBAL VARIABLES ***************

float extVoltage;                  // measured voltage (max. 16.5V)
int batteryLevel;                  // Internal battery percentage
float intVoltage;                  // Internal battery voltage
enum PowerState powerState = INIT; // State of the vehicle battery 0-Off,1-Battery,2-Alternator,3-Init
unsigned int timeToSleep;          // ESP32 deep sleep wakeup time (seconds)
bool vehicleDisabled = false;      // Vehicle disabled status (stored on ESP32 EEPROM Byte 0)
DateTime rtc;                      // RTC datetime object
float gpsHdop;                     // GPS Hdop (Horizontal Dilution of precision)
float gpsAltitude;                 // GPS Altitude
float gpsHeading;                  // GPS Heading
float gpsSpeed;                    // GPS Velocity
float gpsDistance;                 // GPS gpsDistance to Home
double gpsLat;                     // GPS Latitude
double gpsLon;                     // GPS Longitude
short gpsSats;                     // GPS Active Sattelites
short temperature;                 // Temperature
unsigned long gpsTimer = 0UL;      // GPS update timer
unsigned long mainTimer = 0UL;     // Main loop update timer
unsigned long btTimer = 0UL;       // Bluetooth security timer
unsigned long serTimer = 0UL;      // Serial input timer
unsigned long cellTimer = 0UL;     // Cell network timer
char lastRestartReason[16];        // Last restart reason (Stored in RTC EEPROM)
char lastStartup1[48];             // Last startup timestamp & reason (Stored in RTC EEPROM)
char lastStartup2[48];             // Last 2nd startup timestamp (Stored in RTC EEPROM)
char lastStartup3[48];             // Last 3rd startup timestamp (Stored in RTC EEPROM)
char lastTimeAdjust[32];           // Last timestamp or GPS adjusted datetime (Stored in RTC EEPROM)
char lastSleep[48];                // Last recorded sleep time (Stored in RTC EEPROM)
bool gpsFix = false;               // GPS fix status
bool gpsDebug = false;             // GPS Debug Output
bool consoleUnlocked = false;      // Serial unlocked
short consoleTimeout;              // Console timeout after wakeup (seconds)
bool isUpdating = false;           // OTA update mode
char btName[8];                    // Bluetooth Discovery Name
char wifiSsid[7];                  // Wifi SSID for OTA updates (Stored in RTC EEPROM)
char wifiKey[17];                  // Wifi SSID key for OTA updates (Stored in RTC EEPROM)
char consolePassword[8];           // Console Password (Stored in RTC EEPROM)
char smsNumber[13];                // Phone number to send reply SMS to
unsigned long startMicros;         // Function execution timer
unsigned long endMicros;           // Function execution timer
int gpsTimeOffset;                 // Difference between GPS and RTC time in seconds
bool debug;                        // Debug Mode (stored in ESP32 EEPROM Byte 1)
bool cellConnected;                // Cellular network is connected and ready
bool btDeviceConnected = false;    // BT device connected
bool oldDeviceConnected = false;

// ********** FUNCTION DEFINITIONS *************

void gps_check();
void deepsleep();
char serial_read();
void process_input(char incomingByte);
void print_wakeup_reason();
void disable_vehicle();
void int_volt_check();
void ext_volt_check();
void print_status();
void reset_device();
void sms_check();
void overtemp_check();
char* returndatetime();
void printdatetime();
void printDateTime();
char* returnDateTime();
long int syncProvider();
void security_check();
time_t gpsunixtime();
boolean isNumeric(String str);
void serialprint(String buf);
void gotosleep(char* sleepReason);

TaskHandle_t CellLoop;

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = { "EDT", Second, Sun, Mar, 2, -240 };  // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = { "EST", First, Sun, Nov, 2, -300 };   // Standard time = UTC - 5 hours
Timezone usEastern(myDST, mySTD);
TimeChangeRule *tcr;  // pointer to the time change rule, use to get TZ abbrev

RTClib RTC;
DS3231 hwRTC;
uEEPROMLib eeprom(0x57);
Sim800L GSM(MODEM_RX_PIN, MODEM_TX_PIN, MODEM_RESET_PIN);
TinyGPSPlus gps;
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;

// ******************* CLASSES **********************

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      btDeviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      btDeviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 1) 
      {
        if (consoleUnlocked == false && millis() - btTimer < consoleTimeout * 1000)  
        {
          if (rxValue == consolePassword) {
            consoleUnlocked = true;
            serialprint("Console Unlocked\n");
          }
        } 
      } else if (rxValue.length() == 1 && consoleUnlocked) process_input(rxValue[0]);
  }
};

// ******************* SETUP **********************

void setup() {
  Serial.begin(115200);
  pinMode(WAKEUP_PIN, INPUT_PULLDOWN);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  pinMode(RELAY_PIN, OUTPUT);  // Initialize Relay Output Pin
  EEPROM.begin(EEPROM_SIZE);   // Initialize EEPROM
  if (EEPROM.read(0) == 0) {   // Read EEPROM Byte 0 (vehicleDisabled_status)
    vehicleDisabled = false;
  } else {
    vehicleDisabled = true;
  }
  if (EEPROM.read(1) == 0) {  // Read EEPROM Byte 1 (debug mode)
    debug = false;
    consoleUnlocked = false;
  } else {
    debug = true;
    consoleUnlocked = true;
  }
  if (vehicleDisabled) {
    digitalWrite(RELAY_PIN, HIGH);
    serialprint("** Vehicle is Disabled! **\n");
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
  serialprint("Initializing RTC Module...");
  Wire.begin();
  setSyncProvider(syncProvider);   // the function to get the time from the RTC
  if (timeStatus() != timeSet) {
    serialprint("Unable to sync with the RTC\n");
  } else {
    serialprint("RTC has set the system time\n");
  }
  temperature = hwRTC.getTemperature();
  overtemp_check();

  serialprint("Initializing RTC EEPROM...\n");
  eeprom.eeprom_read(16, (byte *)lastTimeAdjust, 32);  // Read last time adjust
  eeprom.eeprom_read(200, (byte *)wifiSsid, 6);  // Read SSID
  eeprom.eeprom_read(208, (byte *)wifiKey, 16);  // Read SSID key
  eeprom.eeprom_read(225, (byte *)consolePassword, 8);  // Read Console password
  eeprom.eeprom_read(233, (byte *)btName, 8);  // Read Bluetooth name
  eeprom.eeprom_read(241, (byte *)smsNumber, 12);  // Read SMS reply number
  eeprom.eeprom_read(253, &consoleTimeout);  // Read Console Timeout
  eeprom.eeprom_read(255, (byte *)lastSleep, 48);  // Read last sleep time

  serialprint("Initializing Hardware Watchdog...\n");
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  serialprint("Initializing GPS Module...\n");
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, HIGH);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Initialize GPS UART

  serialprint("Initializing Bluetooth Module...\n");
  BLEDevice::init("DURANGO");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );              
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  btTimer = millis();

  serialprint("Initialization Complete.\n");
  print_wakeup_reason();
  serialprint((String)"Main loop running on CPU core: " + xPortGetCoreID() + "\n");
  xTaskCreatePinnedToCore(cellular_loop, "CellLoop", 10000, NULL, 1, &CellLoop, 0);
  int_volt_check();
  ext_volt_check();
  print_status();
  gps_check();
}

// ******************* LOOP **********************

void loop() {
  esp_task_wdt_reset();
  if (isUpdating) ArduinoOTA.handle();
  gps_check();
  security_check();
  if (Serial.available()) 
  {
    char incomingByte;
    incomingByte = serial_read();
    if (incomingByte != 0) process_input(incomingByte);
  }
  if (millis() - mainTimer > 5000) 
  {
    temperature = (hwRTC.getTemperature());
    overtemp_check();
    int_volt_check();
    ext_volt_check();
    //print_status();
    if (powerState == OFF || powerState == BATTERY) deepsleep();
    mainTimer = millis();
  }
}

// ******************* FUNCTIONS **********************

void serialbt_check( void * pvParameters ) {
    // disconnecting
    if (!btDeviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        serialprint("Starting BLE advertising...\n");
        oldDeviceConnected = btDeviceConnected;
    }
    // connecting
    if (btDeviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = btDeviceConnected;
    }
  }

void cellular_loop( void * pvParameters ) 
{
  cellConnected = false;
  serialprint((String)"Cellular loop running on CPU core: " + xPortGetCoreID() + "\n");
  
  if (cellactive) 
  {
    serialprint("Initializing SIM800 Module...\n");
    pinMode(MODEM_POWER_PIN, OUTPUT);     // Initialize Sim800 Power Pin
    digitalWrite(MODEM_POWER_PIN, HIGH);  // Power on Sim800L Module
    GSM.begin(9600);                     // Initialize GSM on Sim800L
    serialprint("Waiting for cellular network...\n");
    cellTimer = millis();
    while (!GSM.prepareForSmsReceive())
    {
      if (millis() - cellTimer > 600000) {
        serialprint("Cellular connection timeout. Resetting device...\n");
        reset_device();
      }
    }
    cellConnected = true;
    serialprint("Cellular network connected. Ready to recieve.\n");
  }
  for (;;) {
    sms_check();
  }
}

void serialprint(String buf)
{
  if (debug && consoleUnlocked) 
  {
    char tbuf[buf.length()+1];
    strcpy(tbuf, buf.c_str());
    Serial.print(tbuf);
    if (btDeviceConnected) 
    {
     pTxCharacteristic->setValue(tbuf);
     pTxCharacteristic->notify();
     delay(10);
    }
  }
}

void sms_check() {
  byte index = GSM.checkForSMS();
  if (index != 0) 
  {
    serialprint((String)"SMS Recieved: " + GSM.readSms(index) + "\n");
    disable_vehicle();
    //GSM.delAllSms();  // this is optional
  }
}

long int syncProvider()
{
  return RTC.now().unixtime();
}

void security_check() {
  if (consoleUnlocked == false) {
    if (millis() - btTimer < consoleTimeout * 1000) {
      if (Serial.available()) {
        String teststr = Serial.readString();
        teststr.trim();
        if (teststr == consolePassword) {
          consoleUnlocked = true;
          serialprint("Console Unlocked\n");
        }
      }
    } else if (btDeviceConnected) { //FIXME: detect if BLE is on
      if (debug) Serial.println("Console Unlock Timeout!");
      Serial.end();
      // TODO: Shut off BLE
      WiFi.mode(WIFI_OFF);
    }
  }
}

char serial_read() {
  if (Serial.available() > 0) 
  {
    int incomingByte = Serial.read();
    serialprint((String)incomingByte + "\n");
    return incomingByte;
  } else return 0;
}

/* void readbtname() {
  bool answered = false;
  while (millis() < serTimer + 7000 && answered == false) {
    if (SerialBT.hasClient() && SerialBT.available()) {
      String teststr = SerialBT.readString();  //read until timeout
      teststr.trim();
      answered = true;
      if (debug && consoleUnlocked) {
        //char buff = &btName;
        //strcpy(*buff, teststr);
        //eeprom.eeprom_write(233, (byte *)buff, 8);  // Read Bluetooth name
        //btName = teststr;
        Serial.println((String)"New bluetooth discovery name: " + teststr);
        SerialBT.println((String)"New bluetooth discovery name: " + teststr);
      } else {
        Serial.println("Invalid console timeout value");
        SerialBT.println("Invalid console timeout value");
      }
    } else if (Serial.available()) {
      String teststr = Serial.readString();
      teststr.trim();
      answered = true;
      Serial.println(teststr);
      if (debug && consoleUnlocked) {
        //eeprom.eeprom_write(233, (byte *)teststr, 8);  // Read Bluetooth name
        //btName = teststr;
        Serial.println((String)"New bluetooth discovery name:  " + teststr);
        SerialBT.println((String)"New bluetooth discovery name:  " + teststr);
      } else {
        Serial.println("Invalid console timeout value");
        SerialBT.println("Invalid console timeout value");
      }
    }
  }
  if (answered == false) {
    Serial.println("Input timeout waiting for value");
    SerialBT.println("Input timeout waiting for value");
  }
}

void readtimeout() {
  bool answered = false;
  while (millis() < serTimer + 7000 && answered == false) {
    if (SerialBT.hasClient() && SerialBT.available()) {
      String teststr = SerialBT.readString();  //read until timeout
      teststr.trim();
      answered = true;
      if (isNumeric(teststr)) {
        short nto;
        nto = teststr.toInt();
        eeprom.eeprom_write(253, nto);
        consoleTimeout = nto;
        Serial.println((String)"New console timeout seconds: " + teststr);
        SerialBT.println((String)"New console timeout seconds: " + teststr);
      } else {
        Serial.println("Invalid console timeout value");
        SerialBT.println("Invalid console timeout value");
      }
    } else if (Serial.available()) {
      String teststr = Serial.readString();
      teststr.trim();
      answered = true;
      if (isNumeric(teststr)) {
        short nto;
        nto = teststr.toInt();
        eeprom.eeprom_write(253, nto);
        consoleTimeout = nto;
        Serial.println((String)"New console timeout seconds: " + teststr);
        SerialBT.println((String)"New console timeout seconds: " + teststr);
      } else {
        Serial.println("Invalid console timeout value");
        SerialBT.println("Invalid console timeout value");
      }
    }
  }
  if (answered == false) {
    if (debug && consoleUnlocked) {
      Serial.println("Input timeout waiting for value");
      SerialBT.println("Input timeout waiting for value");
    }
  }
} */

void process_input(char incomingByte) {
  switch (incomingByte) {
    case 104:
      serialprint("**** Device Commands ****\n");
      serialprint("r - Reset Device\n");
      serialprint("s - Device Status\n");
      serialprint("u - OTA Update Mode\n");
      serialprint("d - Toggle Debug Output\n");
      serialprint("g - Toggle GPS Debug Output\n");
      serialprint("x - Disable Vehicle Toggle\n");
      serialprint("l - Lock the console\n");
      serialprint("**** Configuration ****\n");
      serialprint("i - Change OTA Wifi SSID\n");
      serialprint("k - Change OTA Wifi SSID Key\n");
      serialprint("c - Change Console Password\n");
      serialprint("t - Change Console Timeout\n");
      serialprint("b - Change Bluetooth Discovery Name\n");
      serialprint("m - Change SMS Reply Number (+15555551212)\n");
      break;
    case 98:
      serialprint("Enter new bluetooth discovery name: ");
      //readbtname(); FIXME: missing
      break;
    case 116:
      serialprint("Enter new timeout value in seconds: ");
      //readtimeout(); FIXME: missing
      break;
    case 117:
      serialprint("OTA Update mode initilizing...\n");
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSsid, wifiKey);
      while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        serialprint("Wifi Connection Failed!\n");
      }
      ArduinoOTA
      .onStart([]() {
        esp_task_wdt_reset();
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        serialprint((String)"Start isUpdating " + type + "\n");
      })
      .onEnd([]() {
        serialprint("\nEnd\n");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        char* tmp;
        sprintf(tmp, "%u%%\r", (progress / (total / 100)));
        serialprint((String)"Progress: " + tmp + "\n");
        esp_task_wdt_reset();
      })
      .onError([](ota_error_t error) {
        char* tmp;
        sprintf(tmp, "%u", error);
        serialprint((String)"Error[" + tmp + "]: ");
        if (error == OTA_AUTH_ERROR) {
          serialprint("Auth Failed\n");
        }
        else if (error == OTA_BEGIN_ERROR) {
          serialprint("Begin Failed\n");
        }
        else if (error == OTA_CONNECT_ERROR) {
          serialprint("Connect Failed\n");
        }
        else if (error == OTA_RECEIVE_ERROR) {
          serialprint("Receive Failed\n");
        }
        else if (error == OTA_END_ERROR) {
          serialprint("End Failed\n");
        }
      });

      ArduinoOTA.begin();
      serialprint("OTA Ready.\n");
      serialprint("IP address: ");
      serialprint((String)WiFi.localIP() + "\n");
      isUpdating = true;
      break;
    case 108:
      serialprint("Locking Console...\n");
      consoleUnlocked = false;
    case 114:
      serialprint("Resetting device...\n");
      Serial.flush();
      ESP.restart();
      break;
    case 115:
      serialprint("****** status ******\n");
      serialprint("Last ");
      print_wakeup_reason();
      serialprint("RTC Time: ");
      printdatetime();
      serialprint("\n");
      serialprint((String)"Uptime: " + uptime_formatter::getUptime() + "\n");
      serialprint("Last Sleep: ");
      serialprint((String)lastSleep + "\n");
      serialprint((String)"Last Time Adjustment: " + lastTimeAdjust + "\n");
      serialprint((String)"RTC/GPS Time Lag: " + gpsTimeOffset + " Seconds\n");
      serialprint((String)"Temperature: " + temperature + "°C\n");
      serialprint((String)"ESP32 Battery: " + intVoltage + "v (" + batteryLevel + "%)\n");
      serialprint((String)"Vehicle Battery: " + extVoltage + "v\n");
      if (powerState == OFF) {
        serialprint("powerState: No Voltage\n");
      } else if (powerState == BATTERY) {
        serialprint("powerState: Battery Voltage\n");
      } else if (powerState == ALTERNATOR) {
        serialprint("powerState: Alternator Voltage\n");
      }
      if (vehicleDisabled) {
        serialprint("Vehicle Disabled: YES\n");
      } else {
        serialprint("Vehicle Disabled: NO\n");
      }
      serialprint((String)"Aquired Satellites: " + gpsSats + "\n");
      serialprint("Latitude: ");
      serialprint((String)gpsLat); //FIXME: 6 digits
      serialprint("°\n");
      serialprint("Longitude: ");
      serialprint((String)gpsLon); //FIXME: 6 digits
      serialprint("°\n");
      serialprint((String)"Distance to Home: " + gpsDistance + " miles\n");
      serialprint((String)"Precision: " + gpsHdop + " meters\n");
      serialprint((String)"Speed: " + gpsSpeed + " mph\n");
      serialprint((String)"Heading: " + gpsHeading + "°\n");
      serialprint((String)"Altitude: " + gpsAltitude + " feet\n");
      serialprint("Wifi OTA IP address: ");
      serialprint((String)WiFi.localIP() + "\n");
      serialprint((String)"Wifi OTA SSID: " + wifiSsid + "\n");
      serialprint((String)"Wifi OTA wpa2key: " + wifiKey + "\n");
      serialprint((String)"Bluetooth Name: " + btName + "\n");
      serialprint((String)"SMS Reply Number: " + smsNumber + "\n");
      if (Serial) 
      {
        serialprint("Serial: Available\n");
      }
      else 
      {
        serialprint("Serial: Unavailable\n");
      }

      if (btDeviceConnected) 
      {
        serialprint("Bluetooth Serial: Client Connected\n");
      }
      else 
      {
        serialprint("Bluetooth Serial: No Connections\n");
      }
      serialprint((String)"Console Timeout: " + consoleTimeout + " seconds\n");
      if (gpsFix) {
        serialprint("GPS Fix: YES\n");
      } else {
        serialprint("GPS Fix: NO\n");
      }
       if (cellConnected) {
        serialprint("Cellular Connected: YES\n");
      } else {
        serialprint((String)"Cellular Connected: NO\n");
      }
      serialprint("**** end status *****\n");
      break;
    case 100:
      if (debug && consoleUnlocked) {
        debug = false;
        EEPROM.write(1, 0);
        EEPROM.commit();
        serialprint("Debug mode is now: OFF\n");
      } else {
        debug = true;
        EEPROM.write(1, 1);
        EEPROM.commit();
        serialprint("Debug mode is now: ON\n");
      }
      break;
    case 120:
      disable_vehicle();
      break;
    case 103:
      if (gpsDebug) {
        gpsDebug = false;
        serialprint("GPS Debug output is now: OFF\n");
      } else {
        gpsDebug = true;
        serialprint("GPS Debug output is now: ON\n");
      }
      break;
    default:
      serialprint("Invalid command\n");
      break;
  }
}

void gps_check() {
  while (Serial1.available() > 0) gps.encode(Serial1.read());
  if (gps.location.isUpdated())
  {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    if (gpsDebug) {
      serialprint((String)"LOCATION Age: " + gps.location.age() + "ms Raw gpsLat: ");
      serialprint(gps.location.rawLat().negative ? "-" : "+");
      serialprint((String)gps.location.rawLat().deg + "[+" + gps.location.rawLat().billionths + " billionths],  Raw Lon: ");
      serialprint(gps.location.rawLng().negative ? "-" : "+");
      serialprint((String)gps.location.rawLng().deg + "[+" + gps.location.rawLng().billionths + " billionths],  Lat: ");
      serialprint((String)gps.location.lat());  //FIXME: 6 digits
      serialprint(F(" Long="));
      serialprint((String)gps.location.lng() + "\n");
    }
  }
  else if (gps.date.isUpdated())
  {
      if (gpsDebug) serialprint((String)"DATE Fix Age: " + gps.date.age() + "ms Raw: " + gps.date.value() + " Year: " + gps.date.year() + " Month: " + gps.date.month() + " Day=" + gps.date.day() + "\n");
  }
  else if (gps.time.isUpdated())
  {
    rtc = RTC.now();
    time_t gpsEpoch = gpsunixtime();
    time_t rtcEpoch = rtc.unixtime();
    gpsTimeOffset = rtcEpoch - gpsEpoch;
    if (gpsEpoch != 943920000) {
      if (rtcEpoch + 10 < gpsEpoch || rtcEpoch - 10 > gpsEpoch) {
        hwRTC.setHour(gps.time.hour());
        hwRTC.setMinute(gps.time.minute());
        hwRTC.setSecond(gps.time.second());
        hwRTC.setYear(gps.date.year() - 2000);
        hwRTC.setMonth(gps.date.month());
        hwRTC.setDate(gps.date.day());
        char* rdt;
        rdt = returndatetime();
        strcpy(lastTimeAdjust, rdt);
        if (!eeprom.eeprom_write(16, (byte *)lastTimeAdjust, sizeof(lastTimeAdjust))) serialprint("Failed to store data in RTC EEPROM!");
        if (debug && consoleUnlocked) serialprint((String)"RTC time adjusted to GPS: " + rtc.hour() + ":" + rtc.minute() + ":" + rtc.second() + " -> " + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second() + "\n");
      }
    }
    if (gpsDebug) serialprint((String)"TIME Fix Age: " + gps.time.age() + "ms Raw: " + gps.time.value() + " Hour: " + gps.time.hour() + " Minute: " + gps.time.minute() + " Second: " + gps.time.second() + " Hundredths: " + gps.time.centisecond() + "\n");
  }
  else if (gps.speed.isUpdated())
  {
    gpsSpeed = gps.speed.mph();
    if (gpsDebug) Serial.println((String)"SPEED Fix Age: " + gps.speed.age() + "ms Raw: " + gps.speed.value() + " Knots: " + gps.speed.knots() + " MPH: " + gps.speed.mph() + " m/s: " + gps.speed.mps() + " km/h: " + gps.speed.kmph() + "\n");
  }
  else if (gps.course.isUpdated())
  {
    gpsHeading = gps.course.deg();
    if (gpsDebug) Serial.println((String)"COURSE Fix Age: " + gps.course.age() + "ms Raw: " + gps.course.value() + " Deg: " + gps.course.deg() + "\n");
  }
  else if (gps.altitude.isUpdated())
  {
    gpsAltitude = gps.altitude.feet();
    if (gpsDebug) serialprint((String)"ALTITUDE Fix Age: " + gps.altitude.age() + "ms Raw: " + gps.altitude.value() + " Meters: " + gps.altitude.meters() + " Miles: " + gps.altitude.miles() + " KM: " + gps.altitude.kilometers() + " Feet: " + gps.altitude.feet() + "\n");
  }
  else if (gps.satellites.isUpdated())
  {
    gpsSats = gps.satellites.value();
    if (gpsSats > 0 && gpsFix == false) {
      gpsFix = true;
      serialprint((String)"GPS fix aquired with "+gpsSats+" satellites\n");
    } else if (gpsSats == 0) gpsFix = false;
    if (gpsDebug) serialprint((String)"SATELLITES Fix Age: " + gps.satellites.age() + "ms Value: " + gps.satellites.value() + "\n");
  }
  else if (gps.hdop.isUpdated())
  {
    gpsHdop = gps.hdop.hdop();
    if (gpsDebug) serialprint((String)"HDOP Fix Age: " + gps.hdop.age() + "ms raw: " + gps.hdop.value() + " Hdop: " + gps.hdop.hdop() + "\n");
  }
  else if (millis() - gpsTimer > 5000)
  {
    if (gps.location.isValid())
    {
      static const double HOME_LAT = 42.549379, HOME_LON = -82.923508;
      double gpsDistanceToHome =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT,
          HOME_LON);
      double courseToHome =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT,
          HOME_LON);
      gpsDistance = (gpsDistanceToHome / 1000) * 0.62137;
      if (gpsDebug) serialprint((String)"HOME gpsDistance: " + gpsDistance + " mi Course-to=" + courseToHome + " degrees [" + TinyGPSPlus::cardinal(courseToHome) + "]\n");
    }
    if (gpsDebug) serialprint((String)"DIAGS Chars: " + gps.charsProcessed() + " Sentences-with-Fix: " + gps.sentencesWithFix() + " Failed-checksum: " + gps.failedChecksum() + " Passed-checksum: " + gps.passedChecksum() + "\n");
    if (gps.charsProcessed() < 10 && gpsDebug) serialprint("WARNING: No GPS data.  Check wiring.\n");
  }
  gpsTimer = millis();
}

void overtemp_check() {
  if (temperature > OVERTEMP) {
    serialprint((String)temperature + " > " + OVERTEMP + " Over Temperature Shutdown for 10 min\n");
    timeToSleep = 600;
    esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
    gotosleep((char *)"OverTemp " + temperature);
  } else if (temperature < UNDERTEMP) {
    serialprint((String)temperature + " < " + UNDERTEMP + " Under Temperature Shutdown for 10 min\n");
    timeToSleep = 600;
    esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
    gotosleep((char *)"UnderTemp " + temperature);
  }
}

void gotosleep(char* sleepReason) {
  char* rdt;
  rdt = returndatetime();
  strcpy(lastSleep, rdt);
  strcat(lastSleep, " | ");
  strcat(lastSleep, sleepReason);
  eeprom.eeprom_write(255, (byte *)lastSleep, sizeof(lastSleep));  // Write last sleep time & reason
  Serial.flush();
  esp_deep_sleep_start();
}

void deepsleep() {
  if (!isUpdating && !vehicleDisabled) {
    //digitalWrite(GPS_POWER_PIN, LOW);  // Power off GPS module
    if (powerState == OFF) {
      if (intVoltage < 3.3) {
        esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
        timeToSleep = 31536000;
        serialprint("Going to sleep forever due to low internal battery...\n");
        gotosleep("Low Int Batt");
      } else { 
        serialprint("Going to sleep due to low vehicle battery...\n");
        timeToSleep = 3600;
        esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
        gotosleep("Low Car Batt");
      }
    } else if (powerState == BATTERY) {
      serialprint("Going to sleep due to vehicle shutoff...\n");
      timeToSleep = 3600;
      esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
      gotosleep("Vehicle Shutoff");
    }
  } else serialprint("Aborting sleep due to OTA Update or Vehicle Disabled\n");
}

void print_wakeup_reason() {
  if (debug && consoleUnlocked) {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
      case ESP_SLEEP_WAKEUP_EXT0: {
          serialprint("Wakeup caused by external signal using RTC_IO\n");
          break;
        }
      case ESP_SLEEP_WAKEUP_EXT1: {
          serialprint("Wakeup caused by external signal using RTC_CNTL\n");
          break;
        }
      case ESP_SLEEP_WAKEUP_TIMER: {
          serialprint("Wakeup caused by timer\n");
          break;
        }
      case ESP_SLEEP_WAKEUP_TOUCHPAD: {
          serialprint("Wakeup caused by touchpad\n");
          break;
        }
      case ESP_SLEEP_WAKEUP_ULP: {
          serialprint("Wakeup caused by ULP program\n");
          break;
        }
      default: {
          serialprint((String)"Wakeup was not caused by sleep: " + wakeup_reason + "\n");
          break;
        }
    }
  }
}

void disable_vehicle() {
  if (!vehicleDisabled) {
    vehicleDisabled = true;
    EEPROM.write(0, 1);
    EEPROM.commit();
    digitalWrite(RELAY_PIN, HIGH);
    serialprint("Vehicle is now Disabled!\n");
    char* smsmsg;
    char* db;
    char* db2;
    dtostrf(gpsLat, 8, 4, db);
    dtostrf(gpsLon, 8, 4, db2);
    //String smsmsg = "Vehicle vehicleDisabled\n https://maps.google.com/?q="+db+","+db2;
    Serial.println(db);
    if (RESPOND == 1) {
      int smserror = GSM.sendSms(smsNumber, smsmsg);
      serialprint((String)"Sending SMS error code: " + smserror + "\n");
    }
  } else if (vehicleDisabled) {
    vehicleDisabled = false;
    EEPROM.write(0, 0);
    EEPROM.commit();
    digitalWrite(RELAY_PIN, LOW);
    serialprint("Vehicle has been RE-ENABLED\n");
  }
}

void ext_volt_check() {
  float vOut;
  int sample_count = 0;
  float sum = 0;
  while (sample_count < EXT_SAMPLES) {
    sum += analogRead(SENSOR_PIN);
    sample_count++;
    delay(10);
  }
  vOut = ((sum / EXT_SAMPLES) / 4096) * 3.3;
  extVoltage = vOut * FACTOR;
  if (extVoltage < 10) {
    powerState = OFF;
  } else if (extVoltage <= 12.8) {
    powerState = BATTERY;
  } else if (extVoltage > 12.8) {
    powerState = ALTERNATOR;
  }
}

void print_status() {
  if (debug && consoleUnlocked) {
    rtc = RTC.now();
    serialprint((String)"RTC: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " " + rtc.hour() + ":" + rtc.minute() + " | Temp: " + temperature + "°C | Ext Batt: " + extVoltage + "v | Int Batt: " + intVoltage + "v (" + batteryLevel + "%) | " + "powerState: ");
   if (powerState == OFF) {
      serialprint("No Voltage");
    } else if (powerState == BATTERY) {
      serialprint("Battery Voltage");
    } else if (powerState == ALTERNATOR) {
      serialprint("Alternator Voltage");
    }
    if (vehicleDisabled) {
      serialprint(" | Vehicle Disabled: ON\n");
    } else {
      serialprint(" | Vehicle Disabled: OFF\n");
    }
  }
}

void reset_device() {
  if (debug && consoleUnlocked) {
    serialprint("Resetting Device...\n");
    Serial.flush();
  }
  ESP.restart();
}

void int_volt_check() {
  int sample_count = 0;
  int sum = 0;
  while (sample_count < INT_SAMPLES) {
    sum += analogRead(35);
    sample_count++;
    delay(10);
  }
  float raw = sum/10;
  intVoltage = ((((raw/4095)*2)*3.3)*1100)/1000;
  batteryLevel = _min(map(intVoltage, 3.3, 4.2, 0, 100), 100);
  sample_count = 0;
  sum = 0;
}

void old_int_volt_check() {
  int sample_count = 0;
  int sum = 0;
  while (sample_count < INT_SAMPLES) {
    sum += analogRead(35);
    sample_count++;
    delay(10);
  }
  intVoltage = (sum / INT_SAMPLES) / 2350.0;
  batteryLevel = _min(map(sum / INT_SAMPLES, 2000, 2350.0, 0, 100), 100);  //1100
  if (sum / INT_SAMPLES < 1200) batteryLevel = 0;
  sample_count = 0;
  sum = 0;
  intVoltage = intVoltage * 4.20;
}

char* returndatetime() {
  time_t local = usEastern.toLocal(now(), &tcr);
  char* rdt;
  rdt = returnDateTime(local, tcr->abbrev);
  return rdt;
}

void printdatetime() {
  time_t local = usEastern.toLocal(now(), &tcr);
  printDateTime(local, tcr->abbrev);
}

void printDateTime(time_t t, const char *tz)
{
  char buf[32];
  char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(t)));
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
          hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t), tz);
  serialprint(buf);
}

char* returnDateTime(time_t t, const char *tz) {
  static char buf[32];
  char m[4];  // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(t)));
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
          hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t), tz);
  return buf;
}

time_t gpsunixtime() {
  struct tm t;
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min = gps.time.minute();
  t.tm_sec = gps.time.second();
  t.tm_isdst = -1;
  time_t t_of_day;
  t_of_day = mktime(&t);
  return t_of_day;
}

template <class T>
String type_name(const T&)
{
  String s = __PRETTY_FUNCTION__;
  int start = s.indexOf("[with T = ") + 10;
  int stop = s.lastIndexOf(']');
  return s.substring(start, stop);
}

bool isNumeric(String str) {
  for (byte i = 0; i < str.length(); i++)
  {
    if (isDigit(str.charAt(i))) return true;
  }
  return false;
}