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

typedef enum {OFF, BATTERY, ALTERNATOR, INIT} PowerState;

// ************* STRUCTS ***************

typedef struct {
  double Hdop;                     // GPS Hdop (Horizontal Dilution of precision)
  double Altitude;                 // GPS Altitude
  double Heading;                  // GPS Heading
  double Speed;                    // GPS Velocity
  double Distance;                 // GPS Distance to Home
  double Lat;                      // GPS Latitude
  double Lon;                      // GPS Longitude
  short Sats;                      // GPS Active Sattelites
  bool Fix = false;                // GPS fix status
  bool Debug = false;              // GPS Debug Output
  int TimeOffset;                  // Difference between GPS and RTC time in seconds
} Gps;

typedef struct {
  unsigned long Timer = 0UL;      
} Timer;

typedef struct {
  char Timestamp[48];     
} Tswr;  // Timestamp with Reason   

// ************* CONSTANTS ***************

const float FACTOR = 5.5;          // reduction factor of the External Voltage Sensor
const bool cellactive = false;     // Activate cell service
const bool RESPOND = false;        // Respond to SMS
const bool SERIALON = true;        // USB Serial enable
const int UNDERTEMP = -20;         // Undertemp Shutdown temperature in Celsius
const int OVERTEMP = 60;           // Overtemp Shutdown temperature in Celsius
const double HOME_LAT = 42.549379; // Home Location Lattitude
const double HOME_LON = -82.923508;// Home Location Longitude

// ************* GLOBAL VARIABLES ***************

bool vehicleDisabled;              // Vehicle disabled status (stored on ESP32 EEPROM Byte 0)
bool consoleUnlocked = false;      // Serial unlocked
bool isUpdating = false;           // OTA update mode
bool debug;                        // Debug Mode (stored in ESP32 EEPROM Byte 1)
bool cellConnected;                // Cellular network is connected and ready
bool btDeviceConnected = false;    // BT device connected
bool oldDeviceConnected = false;   // Last BT device connected
bool btEnabled = true;             // BT enabled 
double extVoltage;                 // measured voltage (max. 16.5V)
double intVoltage;                 // Internal ESP32 battery voltage
unsigned int timeToSleep;          // ESP32 deep sleep wakeup time (seconds)
short temperature;                 // Temperature
unsigned short batteryLevel;       // Internal battery percentage
unsigned short consoleTimeout;     // Console lock timeout after device wakeup (seconds)
char btName[8];                    // Bluetooth Discovery Name
char wifiSsid[7];                  // Wifi SSID for OTA updates (Stored in RTC EEPROM)
char wifiKey[17];                  // Wifi SSID key for OTA updates (Stored in RTC EEPROM)
char consolePassword[8];           // Console Password (Stored in RTC EEPROM)
char smsNumber[13];                // Phone number to send reply SMS to
char lastRestartReason[16];        // Last restart reason (Stored in RTC EEPROM)
char lastTimeAdjust[32];           // Last timestamp or GPS adjusted datetime (Stored in RTC EEPROM)
Tswr lastStartup1;                 // Last startup timestamp & reason (Stored in RTC EEPROM)
Tswr lastStartup2;                 // Last 2nd startup timestamp (Stored in RTC EEPROM)
Tswr lastStartup3;                 // Last 3rd startup timestamp (Stored in RTC EEPROM)
Tswr lastSleep;                    // Last recorded sleep time (Stored in RTC EEPROM)
PowerState powerState = INIT;      // State of the vehicle battery OFF,BATTERY,ALTERNATOR,INIT
DateTime rtc;                      // RTC datetime object
Gps gps;                           // gps data object
Timer gpsupdate;                   // GPS update timer
Timer mainloop;                    // Main loop update timer
Timer bluetooth;                   // Bluetooth security timer
Timer serial;                      // Serial input timer
Timer cell;                        // Cell network timer

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

TaskHandle_t CellLoop;  // create cpu core 1 task handle

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = { "EDT", Second, Sun, Mar, 2, -240 };  // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = { "EST", First, Sun, Nov, 2, -300 };   // Standard time = UTC - 5 hours
Timezone usEastern(myDST, mySTD);
TimeChangeRule *tcr;  // pointer to the time change rule, use to get TZ abbrev

RTClib RTC;
DS3231 HWRTC;
uEEPROMLib eeprom(0x57);
Sim800L GSM(MODEM_RX_PIN, MODEM_TX_PIN, MODEM_RESET_PIN);
TinyGPSPlus GPS;
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
      if (rxValue.length() > 1) {
        if (consoleUnlocked == false && millis() - bluetooth.Timer < consoleTimeout * 1000)  {
          if (rxValue == consolePassword) {
            consoleUnlocked = true;
            serialprint("Console Unlocked\n\r");
          }
        } 
      } else if (rxValue.length() == 1 && consoleUnlocked) process_input(rxValue[0]);
  }
};

// ******************* SETUP **********************

void setup() {
  if (SERIALON) Serial.begin(115200);
  pinMode(WAKEUP_PIN, INPUT_PULLDOWN);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  pinMode(RELAY_PIN, OUTPUT);  // Initialize Relay Output Pin
  EEPROM.begin(EEPROM_SIZE);   // Initialize EEPROM
  if (EEPROM.read(0) == 0) {   // Read EEPROM Byte 0 (vehicleDisabled_status)
    vehicleDisabled = false; 
  } 
  else {
    vehicleDisabled = true; 
  }
  if (EEPROM.read(1) == 0) {  // Read EEPROM Byte 1 (debug mode)
    debug = false;
    consoleUnlocked = false;
  } 
  else {
    debug = true;
    consoleUnlocked = true;
  }
  if (vehicleDisabled) {
    digitalWrite(RELAY_PIN, HIGH);
    serialprint("** Vehicle is Disabled! **\n\r");
  } 
  else digitalWrite(RELAY_PIN, LOW);
  serialprint("Initializing RTC Module...");
  Wire.begin();
  setSyncProvider(syncProvider);   // the function to get the time from the RTC
  if (timeStatus() != timeSet) serialprint("Unable to sync with the RTC\n\r");
  else serialprint("RTC has set the system time\n\r");
  temperature = HWRTC.getTemperature();
  overtemp_check();

  serialprint("Initializing RTC EEPROM...\n\r");
  eeprom.eeprom_read(16, (byte *)lastTimeAdjust, 32);  // Read last time adjust
  eeprom.eeprom_read(200, (byte *)wifiSsid, 6);  // Read SSID
  eeprom.eeprom_read(208, (byte *)wifiKey, 16);  // Read SSID key
  eeprom.eeprom_read(225, (byte *)consolePassword, 8);  // Read Console password
  eeprom.eeprom_read(233, (byte *)btName, 8);  // Read Bluetooth name
  eeprom.eeprom_read(241, (byte *)smsNumber, 12);  // Read SMS reply number
  eeprom.eeprom_read(253, &consoleTimeout);  // Read Console Timeout
  eeprom.eeprom_read(255, (byte *)lastSleep.Timestamp, 48);  // Read last sleep time

  serialprint("Initializing Hardware Watchdog...\n\r");
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  serialprint("Initializing GPS Module...\n\r");
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, HIGH);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Initialize GPS UART

  serialprint("Initializing Bluetooth Module...\n\r");
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
  bluetooth.Timer = millis();

  serialprint("Initialization Complete.\n\r");
  print_wakeup_reason();
  serialprint((String)"Main loop running on CPU core: " + xPortGetCoreID() + "\n\r");
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
  if (!consoleUnlocked) security_check();
  if (SERIALON && consoleUnlocked) {
    if (Serial.available()) {
      char incomingByte;
      incomingByte = serial_read();
      if (incomingByte != 0) process_input(incomingByte);
    }
  }
  if (millis() - mainloop.Timer > 5000) {
    temperature = (HWRTC.getTemperature());
    overtemp_check();
    int_volt_check();
    ext_volt_check();
    print_status();
    if (powerState == OFF || powerState == BATTERY) deepsleep();
    mainloop.Timer = millis();
  }
}

// ******************* FUNCTIONS **********************

void serialbt_check( void * pvParameters ) {
  if (btEnabled) {
    // disconnecting
    if (!btDeviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        serialprint("Starting BLE advertising...\n\r");
        oldDeviceConnected = btDeviceConnected;
    }
    // connecting
    if (btDeviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = btDeviceConnected;
    }
  }
}

void cellular_loop( void * pvParameters ) {
  cellConnected = false;
  serialprint((String)"Cellular loop running on CPU core: " + xPortGetCoreID() + "\n\r");
  
  if (cellactive) {
    serialprint("Initializing SIM800 Module...\n\r");
    pinMode(MODEM_POWER_PIN, OUTPUT);     // Initialize Sim800 Power Pin
    digitalWrite(MODEM_POWER_PIN, HIGH);  // Power on Sim800L Module
    GSM.begin(9600);                     // Initialize GSM on Sim800L
    serialprint("Waiting for cellular network...\n\r");
    cell.Timer = millis();
    while (!GSM.prepareForSmsReceive()) {
      if (millis() - cell.Timer > 600000) {
        serialprint("Cellular connection timeout. Resetting device...\n\r");
        reset_device();
      }
    }
    cellConnected = true;
    serialprint("Cellular network connected. Ready to recieve.\n\r");
  }
  for (;;) {
    sms_check();
  }
}

void serialprint(String buf) {
  if (debug && consoleUnlocked) {
    char tbuf[buf.length()+1];
    strcpy(tbuf, buf.c_str());
    if (SERIALON) Serial.print(tbuf);
    if (btDeviceConnected) {
      pTxCharacteristic->setValue(tbuf);
      pTxCharacteristic->notify();
      delay(10);
    }
  }
}

void sms_check() {
  byte index = GSM.checkForSMS();
  if (index != 0) {
    serialprint((String)"SMS Recieved: " + GSM.readSms(index) + "\n\r");
    disable_vehicle();
    //GSM.delAllSms();  // this is optional
  }
}

long int syncProvider() {
  return RTC.now().unixtime();
}

void security_check() {
    if (millis() - bluetooth.Timer < consoleTimeout * 1000) {
      if (SERIALON) {
        if (Serial.available()) {
          String teststr = Serial.readString();
          teststr.trim();
          if (teststr == consolePassword) {
            consoleUnlocked = true;
            serialprint("Console Unlocked\n\r");
          }
      }
    } 
    else if (btEnabled) {
      if (debug) serialprint("Console Unlock Timeout!\n\r");
      btEnabled = false;
      pServer->getAdvertising()->stop();
      //pService->stop();
      WiFi.mode(WIFI_OFF);
      if (SERIALON) Serial.end();
    }
  }
}

char serial_read() {
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    serialprint((String)incomingByte + "\n\r");
    return incomingByte;
  } 
  else return 0;
}

/* void readbtname() {
  bool answered = false;
  while (millis() < serial.Timer + 7000 && answered == false) {
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
  while (millis() < serial.Timer + 7000 && answered == false) {
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
      serialprint("**** Device Commands ****\n\r");
      serialprint("r - Reset Device\n\r");
      serialprint("s - Device Status\n\r");
      serialprint("u - OTA Update Mode\n\r");
      serialprint("d - Toggle Debug Output\n\r");
      serialprint("g - Toggle GPS Debug Output\n\r");
      serialprint("x - Disable Vehicle Toggle\n\r");
      serialprint("l - Lock the console\n\r");
      serialprint("**** Configuration ****\n\r");
      serialprint("i - Change OTA Wifi SSID\n\r");
      serialprint("k - Change OTA Wifi SSID Key\n\r");
      serialprint("c - Change Console Password\n\r");
      serialprint("t - Change Console Timeout\n\r");
      serialprint("b - Change Bluetooth Discovery Name\n\r");
      serialprint("m - Change SMS Reply Number (+15555551212)\n\r");
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
      serialprint("OTA Update mode initilizing...\n\r");
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSsid, wifiKey);
      while (WiFi.waitForConnectResult() != WL_CONNECTED) serialprint("Wifi Connection Failed!\n\r");
      ArduinoOTA
      .onStart([]() {
        esp_task_wdt_reset();
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
        else type = "filesystem";
        serialprint((String)"Start isUpdating " + type + "\n\r");
      })
      .onEnd([]() {
        serialprint("\n\rEnd\n\r");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        char* tmp;
        sprintf(tmp, "%u%%\r", (progress / (total / 100)));
        serialprint((String)"Progress: " + tmp + "\n\r");
        esp_task_wdt_reset();
      })
      .onError([](ota_error_t error) {
        char* tmp;
        sprintf(tmp, "%u", error);
        serialprint((String)"Error[" + tmp + "]: ");
        if (error == OTA_AUTH_ERROR) serialprint("Auth Failed\n\r");
        else if (error == OTA_BEGIN_ERROR) serialprint("Begin Failed\n\r");
        else if (error == OTA_CONNECT_ERROR) serialprint("Connect Failed\n\r");
        else if (error == OTA_RECEIVE_ERROR) serialprint("Receive Failed\n\r");
        else if (error == OTA_END_ERROR) serialprint("End Failed\n\r");
      });

      ArduinoOTA.begin();
      serialprint("OTA Ready.\n\r");
      serialprint("IP address: ");
      serialprint((String)WiFi.localIP() + "\n\r");
      isUpdating = true;
      break;
    case 108:
      serialprint("Locking Console...\n\r");
      consoleUnlocked = false;
    case 114:
      serialprint("Resetting device...\n\r");
      Serial.flush();
      ESP.restart();
      break;
    case 115:
      serialprint("****** status ******\n\r");
      serialprint("Last ");
      print_wakeup_reason();
      serialprint("RTC Time: ");
      printdatetime();
      serialprint("\n\r");
      serialprint((String)"Uptime: " + uptime_formatter::getUptime() + "\n\r");
      serialprint("Last Sleep: ");
      serialprint((String)lastSleep.Timestamp + "\n\r");
      serialprint((String)"Last Time Adjustment: " + lastTimeAdjust + "\n\r");
      serialprint((String)"RTC/GPS Time Lag: " + gps.TimeOffset + " Seconds\n\r");
      serialprint((String)"Temperature: " + temperature + "°C\n\r");
      serialprint((String)"ESP32 Battery: " + intVoltage + "v (" + batteryLevel + "%)\n\r");
      serialprint((String)"Vehicle Battery: " + extVoltage + "v\n\r");
      if (powerState == OFF) serialprint("powerState: No Voltage\n\r");
      else if (powerState == BATTERY) serialprint("powerState: Battery Voltage\n\r");
      else if (powerState == ALTERNATOR) serialprint("powerState: Alternator Voltage\n\r");
      if (vehicleDisabled) serialprint("Vehicle Disabled: YES\n\r");
      else serialprint("Vehicle Disabled: NO\n\r");
      serialprint((String)"Aquired Satellites: " + gps.Sats + "\n\r");
      serialprint("Latitude: ");
      serialprint((String)gps.Lat); //FIXME: 6 digits
      serialprint("°\n\r");
      serialprint("Longitude: ");
      serialprint((String)gps.Lon); //FIXME: 6 digits
      serialprint("°\n\r");
      serialprint((String)"Distance to Home: " + gps.Distance + " miles\n\r");
      serialprint((String)"Precision: " + gps.Hdop + " meters\n\r");
      serialprint((String)"Speed: " + gps.Speed + " mph\n\r");
      serialprint((String)"Heading: " + gps.Heading + "°\n\r");
      serialprint((String)"Altitude: " + gps.Altitude + " feet\n\r");
      serialprint("Wifi OTA IP address: ");
      serialprint((String)WiFi.localIP() + "\n\r");
      serialprint((String)"Wifi OTA SSID: " + wifiSsid + "\n\r");
      serialprint((String)"Wifi OTA wpa2key: " + wifiKey + "\n\r");
      serialprint((String)"Bluetooth Name: " + btName + "\n\r");
      serialprint((String)"SMS Reply Number: " + smsNumber + "\n\r");
      if (Serial) serialprint("Serial: Available\n\r");
      else serialprint("Serial: Unavailable\n\r");
      if (btDeviceConnected) serialprint("Bluetooth Serial: Client Connected\n\r");
      else serialprint("Bluetooth Serial: No Connections\n\r");
      serialprint((String)"Console Timeout: " + consoleTimeout + " seconds\n\r");
      if (gps.Fix) serialprint("GPS Fix: YES\n\r");
      else serialprint("GPS Fix: NO\n\r");
      if (cellConnected) serialprint("Cellular Connected: YES\n\r");
      else serialprint("Cellular Connected: NO\n\r");
      if (debug) serialprint("Debug Mode: ON\n\r");
      else serialprint("Debug Mode: OFF\n\r");
      serialprint("**** end status *****\n\r");
      break;
    case 100:
      if (debug && consoleUnlocked) {
        debug = false;
        EEPROM.write(1, 0);
        EEPROM.commit();
        serialprint("Debug mode is now: OFF\n\r");
      } else {
        debug = true;
        EEPROM.write(1, 1);
        EEPROM.commit();
        serialprint("Debug mode is now: ON\n\r");
      }
      break;
    case 120:
      disable_vehicle();
      break;
    case 103:
      if (gps.Debug) {
        gps.Debug = false;
        serialprint("GPS Debug output is now: OFF\n\r");
      } else {
        gps.Debug = true;
        serialprint("GPS Debug output is now: ON\n\r");
      }
      break;
    default:
      serialprint("Invalid command\n\r");
      break;
  }
}

void gps_check() {
  while (Serial1.available() > 0) GPS.encode(Serial1.read());
  if (GPS.location.isUpdated()) {
    gps.Lat = GPS.location.lat();
    gps.Lon = GPS.location.lng();
    if (gps.Debug) {
      serialprint((String)"LOCATION Age: " + GPS.location.age() + "ms Raw Lat: ");
      serialprint(GPS.location.rawLat().negative ? "-" : "+");
      serialprint((String)GPS.location.rawLat().deg + "[+" + GPS.location.rawLat().billionths + " billionths],  Raw Lon: ");
      serialprint(GPS.location.rawLng().negative ? "-" : "+");
      serialprint((String)GPS.location.rawLng().deg + "[+" + GPS.location.rawLng().billionths + " billionths],  Lat: ");
      serialprint((String)GPS.location.lat());  //FIXME: 6 digits
      serialprint(F(" Long="));
      serialprint((String)GPS.location.lng() + "\n\r"); 
    } 
  }
  else if (GPS.date.isUpdated()) {
      if (gps.Debug) serialprint((String)"DATE Fix Age: " + GPS.date.age() + "ms Raw: " + GPS.date.value() + " Year: " + GPS.date.year() + " Month: " + GPS.date.month() + " Day=" + GPS.date.day() + "\n\r"); 
  }
  else if (GPS.time.isUpdated()) {
    rtc = RTC.now();
    time_t gpsEpoch = gpsunixtime();
    time_t rtcEpoch = rtc.unixtime();
    gps.TimeOffset = rtcEpoch - gpsEpoch;
    if (gpsEpoch != 943920000) {
      if (rtcEpoch + 10 < gpsEpoch || rtcEpoch - 10 > gpsEpoch) {
        HWRTC.setHour(GPS.time.hour());
        HWRTC.setMinute(GPS.time.minute());
        HWRTC.setSecond(GPS.time.second());
        HWRTC.setYear(GPS.date.year() - 2000);
        HWRTC.setMonth(GPS.date.month());
        HWRTC.setDate(GPS.date.day());
        char* rdt;
        rdt = returndatetime();
        strcpy(lastTimeAdjust, rdt);
        if (!eeprom.eeprom_write(16, (byte *)lastTimeAdjust, sizeof(lastTimeAdjust))) serialprint("Failed to store data in RTC EEPROM!");
        if (debug && consoleUnlocked) serialprint((String)"RTC time adjusted to GPS: " + rtc.hour() + ":" + rtc.minute() + ":" + rtc.second() + " -> " + GPS.time.hour() + ":" + GPS.time.minute() + ":" + GPS.time.second() + "\n\r");
      }
    }
    if (gps.Debug) serialprint((String)"TIME Fix Age: " + GPS.time.age() + "ms Raw: " + GPS.time.value() + " Hour: " + GPS.time.hour() + " Minute: " + GPS.time.minute() + " Second: " + GPS.time.second() + " Hundredths: " + GPS.time.centisecond() + "\n\r");
  } 
  else if (GPS.speed.isUpdated()) {
    gps.Speed = GPS.speed.mph();
    if (gps.Debug) serialprint((String)"SPEED Fix Age: " + GPS.speed.age() + "ms Raw: " + GPS.speed.value() + " Knots: " + GPS.speed.knots() + " MPH: " + GPS.speed.mph() + " m/s: " + GPS.speed.mps() + " km/h: " + GPS.speed.kmph() + "\n\r");
  }
  else if (GPS.course.isUpdated()) {
    gps.Heading = GPS.course.deg();
    if (gps.Debug) serialprint((String)"COURSE Fix Age: " + GPS.course.age() + "ms Raw: " + GPS.course.value() + " Deg: " + GPS.course.deg() + "\n\r");
  }
  else if (GPS.altitude.isUpdated()) {
    gps.Altitude = GPS.altitude.feet();
    if (gps.Debug) serialprint((String)"ALTITUDE Fix Age: " + GPS.altitude.age() + "ms Raw: " + GPS.altitude.value() + " Meters: " + GPS.altitude.meters() + " Miles: " + GPS.altitude.miles() + " KM: " + GPS.altitude.kilometers() + " Feet: " + GPS.altitude.feet() + "\n\r");
  }
  else if (GPS.satellites.isUpdated()) {
    gps.Sats = GPS.satellites.value();
    if (gps.Sats > 0 && !gps.Fix) {
      gps.Fix = true;
      serialprint((String)"GPS fix aquired with "+gps.Sats+" satellites\n\r");
    } 
    else if (gps.Sats < 1) gps.Fix = false;
    if (gps.Debug) serialprint((String)"SATELLITES Fix Age: " + GPS.satellites.age() + "ms Value: " + GPS.satellites.value() + "\n\r");
  }
  else if (GPS.hdop.isUpdated()) {
    gps.Hdop = GPS.hdop.hdop();
    if (gps.Debug) serialprint((String)"HDOP Fix Age: " + GPS.hdop.age() + "ms raw: " + GPS.hdop.value() + " Hdop: " + GPS.hdop.hdop() + "\n\r");
  }
  else if (millis() - gpsupdate.Timer > 5000) {
    if (GPS.location.isValid()) {
      double gpsDistanceToHome = TinyGPSPlus::distanceBetween(GPS.location.lat(), GPS.location.lng(), HOME_LAT, HOME_LON);
      double courseToHome = TinyGPSPlus::courseTo(GPS.location.lat(), GPS.location.lng(), HOME_LAT, HOME_LON);
      gps.Distance = (gpsDistanceToHome / 1000) * 0.62137;
      if (gps.Debug) serialprint((String)"HOME Distance: " + gps.Distance + " mi Course-to=" + courseToHome + " degrees [" + TinyGPSPlus::cardinal(courseToHome) + "]\n\r");
    }
    if (gps.Debug) serialprint((String)"DIAGS Chars: " + GPS.charsProcessed() + " Sentences-with-Fix: " + GPS.sentencesWithFix() + " Failed-checksum: " + GPS.failedChecksum() + " Passed-checksum: " + GPS.passedChecksum() + "\n\r");
    if (GPS.charsProcessed() < 10 && gps.Debug) serialprint("WARNING: No GPS data.  Check wiring.\n\r");
  }
  gpsupdate.Timer = millis();
}

void overtemp_check() {
  if (temperature > OVERTEMP) {
    serialprint((String)temperature + " > " + OVERTEMP + " Over Temperature Shutdown for 10 min\n\r");
    timeToSleep = 600;
    esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
    gotosleep((char *)"OverTemp " + temperature);
  } 
  else if (temperature < UNDERTEMP) {
    serialprint((String)temperature + " < " + UNDERTEMP + " Under Temperature Shutdown for 10 min\n\r");
    timeToSleep = 600;
    esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
    gotosleep((char *)"UnderTemp " + temperature);
  }
}

void gotosleep(char* sleepReason) {
  char* rdt;
  rdt = returndatetime();
  strcpy(lastSleep.Timestamp, rdt);
  strcat(lastSleep.Timestamp, " | ");
  strcat(lastSleep.Timestamp, sleepReason);
  eeprom.eeprom_write(255, (byte *)lastSleep.Timestamp, sizeof(lastSleep.Timestamp));  // Write last sleep time & reason
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
        serialprint("Going to sleep forever due to low internal battery...\n\r");
        gotosleep("Low Int Batt");
      } 
      else { 
        serialprint("Going to sleep due to low vehicle battery...\n\r");
        timeToSleep = 3600;
        esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
        gotosleep("Low Car Batt");
      }
    } 
    else if (powerState == BATTERY) {
      serialprint("Going to sleep due to vehicle shutoff...\n\r");
      timeToSleep = 3600;
      esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
      gotosleep("Vehicle Shutoff");
    }
  } else serialprint("Aborting sleep due to OTA Update or Vehicle Disabled\n\r");
}

void print_wakeup_reason() {
  if (debug && consoleUnlocked) {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
      case ESP_SLEEP_WAKEUP_EXT0: {
          serialprint("Wakeup caused by external signal using RTC_IO\n\r");
          break;
        }
      case ESP_SLEEP_WAKEUP_EXT1: {
          serialprint("Wakeup caused by external signal using RTC_CNTL\n\r");
          break;
        }
      case ESP_SLEEP_WAKEUP_TIMER: {
          serialprint("Wakeup caused by timer\n\r");
          break;
        }
      case ESP_SLEEP_WAKEUP_TOUCHPAD: {
          serialprint("Wakeup caused by touchpad\n\r");
          break;
        }
      case ESP_SLEEP_WAKEUP_ULP: {
          serialprint("Wakeup caused by ULP program\n\r");
          break;
        }
      default: {
          serialprint((String)"Wakeup was not caused by sleep: " + wakeup_reason + "\n\r");
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
    serialprint("Vehicle is now Disabled!\n\r");
    char* smsmsg;
    char* db;
    char* db2;
    dtostrf(gps.Lat, 8, 4, db);
    dtostrf(gps.Lon, 8, 4, db2);
    smsmsg = (char *)"Vehicle Disabled\n\rhttps://maps.google.com/?q=";
    strcat(smsmsg, db);
    strcat(smsmsg, ",");
    strcat(smsmsg, db2);
    if (RESPOND == 1) {
      int smserror = GSM.sendSms(smsNumber, smsmsg);
      serialprint((String)"Sending SMS error code: " + smserror + "\n\r");
    }
  } 
  else {
    vehicleDisabled = false;
    EEPROM.write(0, 0);
    EEPROM.commit();
    digitalWrite(RELAY_PIN, LOW);
    serialprint("Vehicle has been RE-ENABLED\n\r");
  }
}

void ext_volt_check() {
  double vOut;
  int sample_count = 0;
  double sum = 0;
  while (sample_count < EXT_SAMPLES) {
    sum += analogRead(SENSOR_PIN);
    sample_count++;
    delay(10);
  }
  vOut = ((sum / EXT_SAMPLES) / 4096) * 3.3;
  extVoltage = vOut * FACTOR;
  if (extVoltage < 10) powerState = OFF;
  else if (extVoltage <= 12.8) powerState = BATTERY;
  else powerState = ALTERNATOR;
}

void print_status() {
  if (debug && consoleUnlocked) {
    rtc = RTC.now();
    serialprint((String)"RTC: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " " + rtc.hour() + ":" + rtc.minute() + " | Temp: " + temperature + "°C | Ext Batt: " + extVoltage + "v | Int Batt: " + intVoltage + "v (" + batteryLevel + "%) | " + "powerState: ");
    if (powerState == OFF) serialprint("No Voltage");
    else if (powerState == BATTERY) serialprint("Battery Voltage");
    else if (powerState == ALTERNATOR) serialprint("Alternator Voltage");
    if (vehicleDisabled) serialprint(" | Vehicle Disabled: ON\n\r");
    else serialprint(" | Vehicle Disabled: OFF\n\r");
  }
}

void reset_device() {
  serialprint("Resetting Device...\n\r");
  Serial.flush();
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
  double raw = sum / 10;
  intVoltage = ((((raw/4095)*2)*3.3)*1100)/1000;
  batteryLevel = _min(map(intVoltage, 3.3, 4.2, 0, 100), 100);
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
  t.tm_year = GPS.date.year() - 1900;
  t.tm_mon = GPS.date.month() - 1;
  t.tm_mday = GPS.date.day();
  t.tm_hour = GPS.time.hour();
  t.tm_min = GPS.time.minute();
  t.tm_sec = GPS.time.second();
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
  for (byte i = 0; i < str.length(); i++) {
    if (isDigit(str.charAt(i))) return true;
  }
  return false;
}