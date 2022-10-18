/*
  Galaxy Vehicle Security Module

  https://github.com/cosmicc/GVSM
  Created by: ianperry99@gmail.com
*/

//#include <Wire.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <SoftwareSerial.h>
#include <Sim800L.h>
#include <DS3231.h>
#include <uEEPROMLib.h>
#include <TinyGPSPlus.h>
#include <uptime_formatter.h>
#include <Timezone.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define MODEM_RX 26                // Sim800 UART RX pin
#define MODEM_TX 27                // Sim800 UART TX pin
#define MODEM_RESET 5              // Sim800 Reset Pin
#define MODEM_POWER_ON 23          // Sim800 Power on Pin
#define GPS_POWER_ON 4             // GPS Module Power Pin
#define GPS_RX 33                  // GPS UART RX PIN
#define GPS_TX 32                  // GPS UART TX PIN
#define GPS_BAUD 9600              // GPS UART Speed
#define RELAY_PIN 13               // Relay trigger pin
#define SENSOR_PIN 34              // Voltage sensor pin
#define WAKEUP_PIN 27              // Deep sleep external wakeup pin
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define EEPROM_SIZE 2              // ESP32 EEPROM save usage bytes
#define INT_SAMPLES 10             // Internal battery voltage samples
#define EXT_SAMPLES 10             // External battery voltage samples
#define WDT_TIMEOUT 10             // Hardware watchdog timeout (seconds)
#define OVERTEMP 60                // Overtemp Shutdown temperature in Celsius
#define UNDERTEMP -20              // Undertemp Shutdown temperature in Celsius
#define RESPOND 0                  // Respond to SMS 0-No,1-Yes

const float factor = 5.5;          // reduction factor of the External Voltage Sensor
const bool cellactive = false;     // Activate cell service (production true)

float ext_voltage;                 // measured voltage (max. 16.5V)
int batteryLevel;                  // Internal battery percentage
float int_voltage;                 // Internal battery voltage
short powerstate = 3;              // State of the vehicle battery 0-Off,1-Battery,2-Alternator,3-Init
unsigned int TIME_TO_SLEEP;        // ESP32 deep sleep wakeup time (seconds)
bool Disabled = false;             // Vehicle disabled status (stored on ESP32 EEPROM Byte 0)
DateTime rtc;                      // RTC datetime object
float hdop;                        // GPS HDOP (Horizontal Dilution of precision)
float altitude;                    // GPS Altitude
float heading;                     // GPS Heading
float speed;                       // GPS Velocity
float distance;                    // GPS Distance to Home
double lat;                        // GPS Latitude
double lon;                        // GPS Longitude
short sats;                        // GPS Active Sattelites
short temp;                        // Temperature
unsigned long last = 0UL;          // GPS update timer
unsigned long mainloop = 0UL;      // Main loop update timer
unsigned long bttimer = 0UL;       // Bluetooth security timer
unsigned long intimer = 0UL;       // Serial input timer
unsigned long celltimer = 0UL;     // Cell network timer
char last_restart_reason[16];      // Last restart reason (Stored in RTC EEPROM)
char last_startup1[48];            // Last startup timestamp & reason (Stored in RTC EEPROM)
char last_startup2[48];            // Last 2nd startup timestamp (Stored in RTC EEPROM)
char last_startup3[48];            // Last 3rd startup timestamp (Stored in RTC EEPROM)
char last_time_adjust[32];         // Last timestamp or GPS adjusted datetime
char last_sleep[48];               // Last recorded sleep time
bool gpsfix = false;               // GPS fix status
bool gpsdebug = false;             // GPS Debug Output
bool unlocked = false;             // Serial unlocked
short ctimeout;                    // Console timeout after wakeup (seconds)
bool updating = false;             // OTA update mode
char BTname[8];                    // Bluetooth Discovery Name
char ssid[7];                      // Wifi SSID for OTA updates (Stored in RTC EEPROM)
char ssidkey[17];                  // Wifi SSID key for OTA updates (Stored in RTC EEPROM)
char consolepw[8];                 // Console Password (Stored in RTC EEPROM)
char smsnumber[13];                // Phone number to send reply SMS to
unsigned long startMicros;         // Function execution timer
unsigned long endMicros;           // Function execution timer
bool debug;                        // Debug Mode (stored in ESP32 EEPROM Byte 1)
bool cellconnected;                   // Cellular network is connected and ready

void gps_check();
void deepsleep();
char serial_read();
char serialbt_read();
void process_read(char incomingByte);
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
void serialprint(char* buf);

TaskHandle_t CellLoop;

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = { "EDT", Second, Sun, Mar, 2, -240 };  // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = { "EST", First, Sun, Nov, 2, -300 };   // Standard time = UTC - 5 hours
Timezone usEastern(myDST, mySTD);
TimeChangeRule *tcr;  // pointer to the time change rule, use to get TZ abbrev

RTClib RTC;
DS3231 hwRTC;
uEEPROMLib eeprom(0x57);
Sim800L GSM(MODEM_RX, MODEM_TX, MODEM_RESET);
TinyGPSPlus gps;
BluetoothSerial SerialBT;

// ******************* SETUP **********************

void setup() {
  Serial.begin(115200);
  char sbuf[50];
  pinMode(WAKEUP_PIN, INPUT_PULLDOWN);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  pinMode(RELAY_PIN, OUTPUT);  // Initialize Relay Output Pin
  EEPROM.begin(EEPROM_SIZE);   // Initialize EEPROM
  if (EEPROM.read(0) == 0) {   // Read EEPROM Byte 0 (disabled_status)
    Disabled = false;
  } else {
    Disabled = true;
  }
  if (EEPROM.read(1) == 0) {  // Read EEPROM Byte 1 (debug mode)
    debug = false;
    unlocked = false;
  } else {
    debug = true;
    unlocked = true;
  }
  if (Disabled) {
    digitalWrite(RELAY_PIN, HIGH);
    if (debug && unlocked) {
      Serial.println("** Vehicle is DISABLED! **");
    }
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

  if (debug && unlocked) {
    Serial.print("Initializing RTC Module...");
  }
  Wire.begin();
  setSyncProvider(syncProvider);   // the function to get the time from the RTC
  if (timeStatus() != timeSet) {
    if (debug && unlocked) {
      Serial.println("Unable to sync with the RTC");
    }
  } else {
    if (debug && unlocked) {
      Serial.println("RTC has set the system time");
    }
  }
  temp = hwRTC.getTemperature();
  overtemp_check();

  if (debug && unlocked) {
    Serial.println("Initializing RTC EEPROM...");
  }
  eeprom.eeprom_read(16, (byte *)last_time_adjust, 32);  // Read last time adjust
  eeprom.eeprom_read(200, (byte *)ssid, 6);  // Read SSID
  eeprom.eeprom_read(208, (byte *)ssidkey, 16);  // Read SSID key
  eeprom.eeprom_read(225, (byte *)consolepw, 8);  // Read Console password
  eeprom.eeprom_read(233, (byte *)BTname, 8);  // Read Bluetooth name
  eeprom.eeprom_read(241, (byte *)smsnumber, 12);  // Read SMS reply number
  eeprom.eeprom_read(253, &ctimeout);  // Read Console Timeout
  eeprom.eeprom_read(255, (byte *)last_sleep, 48);  // Read last sleep time
  if (debug && unlocked) {
    Serial.println("Initializing Hardware Watchdog...");
  }
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  if (debug && unlocked) {
    Serial.println("Initializing GPS Module...");
  }
  pinMode(GPS_POWER_ON, OUTPUT);
  digitalWrite(GPS_POWER_ON, HIGH);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // Initialize GPS UART



  if (debug && unlocked) {
    Serial.println("Initializing Bluetooth Module...");
  }
  SerialBT.begin(BTname); //Bluetooth device name
  bttimer = millis();

  if (debug && unlocked) {
    serialprint("Initialization Complete.\n");
    print_wakeup_reason();
  }
  xTaskCreatePinnedToCore(
    cellular_loop,   /* Task function. */
    "CellLoop",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &CellLoop,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

  int_volt_check();
  ext_volt_check();
  if (debug && unlocked) {
    print_status();
  }
  gps_check();
}

// ******************* LOOP **********************

void loop() {
  esp_task_wdt_reset();
  if (updating) {
    ArduinoOTA.handle();
  }
  gps_check();
  security_check();
  char incomingByte;
  incomingByte = serial_read();
  if (incomingByte != 0) {
    process_read(incomingByte);
  }
  incomingByte = serialbt_read();
  if (incomingByte != 0) {
    process_read(incomingByte);
  }
  if (millis() - mainloop > 5000) {
    temp = (hwRTC.getTemperature());
    overtemp_check();
    int_volt_check();
    ext_volt_check();
    print_status();
    if (powerstate < 2) {
      deepsleep();
    }
    mainloop = millis();
  }
}

// ******************* FUNCTIONS **********************

void cellular_loop( void * pvParameters ) {
  cellconnected = false;
  if (debug && unlocked) {
    Serial.println((String)"Cellular functions running on core: " + xPortGetCoreID());
    SerialBT.println((String)"Cellular functions running on core: " + xPortGetCoreID());
    Serial.println((String)"All other functions running on core: 1" );
    SerialBT.println((String)"All other functions running on core: 1" );
  }
    if (cellactive) {
      if (debug && unlocked) {
      Serial.println("Initializing SIM800 Module...");
      }
      pinMode(MODEM_POWER_ON, OUTPUT);     // Initialize Sim800 Power Pin
      digitalWrite(MODEM_POWER_ON, HIGH);  // Power on Sim800L Module
      GSM.begin(9600);                     // Initialize GSM on Sim800L
  
    if (debug && unlocked) {
      Serial.println("Waiting for cellular network...");
      SerialBT.println("Waiting for cellular network...");
    }
    celltimer = millis();
    while (!GSM.prepareForSmsReceive())
    {
      if (millis() - celltimer > 600000) {
        if (debug && unlocked) {
          Serial.println("Cellular connection timeout. Resetting device...");
          SerialBT.println("Cellular connection timeout, Resetting device...");
        }
        reset_device();
      }
    }
    cellconnected = true;
    if (debug && unlocked) {
      Serial.println("Cellular network connected. Ready to recieve.");
      SerialBT.println("Cellular network connected. Ready to recieve.");
    }
  }
  for (;;) {
    sms_check();
  }
}

void serialprint(char* buf)
{
  Serial.print(buf);
  SerialBT.print(buf);
}

void sms_check() {
  byte index = GSM.checkForSMS();
  if (index != 0) {
    if (debug && unlocked) {
      Serial.print((String)"SMS Recieved: " + GSM.readSms(index));
      SerialBT.print((String)"SMS Recieved: " + GSM.readSms(index));
    }
    disable_vehicle();
    //GSM.delAllSms();  // this is optional
  }
}

long int syncProvider()
{
  return RTC.now().unixtime();
}

void security_check() {
  if (unlocked == false) {
    if (millis() - bttimer < ctimeout * 1000) {
      if (SerialBT.hasClient() && SerialBT.available()) {
        String teststr = SerialBT.readString();  //read until timeout
        teststr.trim();
        if (teststr == consolepw) {
          unlocked = true;
          serialprint("Console Unlocked\n");
        }
      } else if (Serial.available()) {
        String teststr = Serial.readString();
        teststr.trim();
        if (teststr == consolepw) {
          unlocked = true;
          serialprint("Console Unlocked\n");
        }
      }
    } else if (SerialBT.isReady()) {
      if (debug) Serial.println("Console Unlock Timeout!");
      SerialBT.end();
      Serial.end();
      WiFi.mode(WIFI_OFF);
      btStop();
    }
  }
}

char serialbt_read() {
  if (SerialBT.available() > 0) {
    int incomingByte = SerialBT.read();
    if (debug && unlocked) {
      Serial.println(incomingByte);
      SerialBT.println(incomingByte);
    }
    return incomingByte;
  } else {
    return 0;
  }
}

char serial_read() {
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (debug && unlocked) {
      Serial.println(incomingByte);
      SerialBT.println(incomingByte);
    }
    return incomingByte;
  } else {
    return 0;
  }
}

void readbtname() {
  bool answered = false;
  while (millis() < intimer + 7000 && answered == false) {
    if (SerialBT.hasClient() && SerialBT.available()) {
      String teststr = SerialBT.readString();  //read until timeout
      teststr.trim();
      answered = true;
      if (debug && unlocked) {
        //char buff = &BTname;
        //strcpy(*buff, teststr);
        //eeprom.eeprom_write(233, (byte *)buff, 8);  // Read Bluetooth name
        //BTname = teststr;
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
      if (debug && unlocked) {
        //eeprom.eeprom_write(233, (byte *)teststr, 8);  // Read Bluetooth name
        //BTname = teststr;
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
  while (millis() < intimer + 7000 && answered == false) {
    if (SerialBT.hasClient() && SerialBT.available()) {
      String teststr = SerialBT.readString();  //read until timeout
      teststr.trim();
      answered = true;
      if (isNumeric(teststr)) {
        short nto;
        nto = teststr.toInt();
        eeprom.eeprom_write(253, nto);
        ctimeout = nto;
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
        ctimeout = nto;
        Serial.println((String)"New console timeout seconds: " + teststr);
        SerialBT.println((String)"New console timeout seconds: " + teststr);
      } else {
        Serial.println("Invalid console timeout value");
        SerialBT.println("Invalid console timeout value");
      }
    }
  }
  if (answered == false) {
    if (debug && unlocked) {
      Serial.println("Input timeout waiting for value");
      SerialBT.println("Input timeout waiting for value");
    }
  }
}

void process_read(char incomingByte) {
  switch (incomingByte) {
    case 104:
      serialprint("**** Device Commands ****\n");
      serialprint("r - Reset Device\n");
      SerialBT.println("r - Reset Device");
      Serial.println("s - Device Status");
      SerialBT.println("s - Device Status");
      Serial.println("u - OTA Update Mode");
      SerialBT.println("u - OTA Update Mode");
      Serial.println("d - Toggle Debug Output");
      SerialBT.println("d - Toggle Debug Output");
      Serial.println("g - Toggle GPS Debug Output");
      SerialBT.println("g - Toggle GPS Debug Output");
      Serial.println("x - Disable Vehicle Toggle");
      SerialBT.println("x - Disable Vehicle Toggle");
      Serial.println("l - Lock the console");
      SerialBT.println("l - Lock the console");
      Serial.println("**** Configuration ****");
      SerialBT.println("**** Configuration ****");
      Serial.println("i - Change OTA Wifi SSID");
      Serial.println("k - Change OTA Wifi SSID Key");
      Serial.println("c - Change Console Password");
      Serial.println("t - Change Console Timeout");
      Serial.println("b - Change Bluetooth Discovery Name");
      Serial.println("m - Change SMS Reply Number (+15555551212)");
      break;
    case 98:
      Serial.println("Enter new bluetooth discovery name: ");
      SerialBT.println("Enter new bluetooth discovery name: ");
      readbtname();
      break;
    case 116:
      Serial.println("Enter new timeout value in seconds: ");
      SerialBT.println("Enter new timeout value in seconds: ");
      readtimeout();
      break;
    case 117:
      Serial.println("OTA Update mode initilizing...");
      SerialBT.println("OTA Update mode initilizing...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, ssidkey);
      while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Wifi Connection Failed!");
        SerialBT.println("Wifi Connection Failed!");
      }
      ArduinoOTA
      .onStart([]() {
        esp_task_wdt_reset();
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        Serial.println((String)"Start updating " + type);
        SerialBT.println((String)"Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
        SerialBT.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        SerialBT.printf("Progress: %u%%\r", (progress / (total / 100)));
        esp_task_wdt_reset();
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        SerialBT.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Serial.println("Auth Failed");
          SerialBT.println("Auth Failed");
        }
        else if (error == OTA_BEGIN_ERROR) {
          Serial.println("Begin Failed");
          SerialBT.println("Begin Failed");
        }
        else if (error == OTA_CONNECT_ERROR) {
          Serial.println("Connect Failed");
          SerialBT.println("Connect Failed");
        }
        else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("Receive Failed");
          SerialBT.println("Receive Failed");
        }
        else if (error == OTA_END_ERROR) {
          Serial.println("End Failed");
          SerialBT.println("End Failed");
        }
      });

      ArduinoOTA.begin();
      Serial.println("OTA Ready.");
      SerialBT.println("OTA Ready.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      SerialBT.print("IP address: ");
      SerialBT.println(WiFi.localIP());
      updating = true;
      break;
    case 108:
      Serial.println("Locking Console...");
      SerialBT.println("Locking Console...");
      unlocked = false;
    case 114:
      Serial.println("Resetting device...");
      SerialBT.println("Resetting device...");
      Serial.flush();
      SerialBT.flush();
      ESP.restart();
      break;
    case 115:
      Serial.println("****** status ******");
      SerialBT.println("****** status ******");

      Serial.print("Last ");
      SerialBT.print("Last ");
      print_wakeup_reason();

      Serial.print("RTC Time: ");
      SerialBT.print("RTC Time: ");
      printdatetime();

      Serial.println("Uptime: " + uptime_formatter::getUptime());
      SerialBT.println("Uptime: " + uptime_formatter::getUptime());

      Serial.print("Last Sleep: ");
      Serial.println(last_sleep);
      SerialBT.print("Last Sleep: ");
      SerialBT.println(last_sleep);

      Serial.println((String)"Last Time Adjustment: " + last_time_adjust);
      SerialBT.println((String)"Last Time Adjustment: " + last_time_adjust);

      Serial.println((String)"Temperature: " + temp + "°C");
      SerialBT.println((String)"Temperature: " + temp + "°C");

      Serial.println((String)"ESP32 Battery: " + int_voltage + "v (" + batteryLevel + "%)");
      SerialBT.println((String)"ESP32 Battery: " + int_voltage + "v (" + batteryLevel + "%)");

      Serial.println((String)"Vehicle Battery: " + ext_voltage + "v");
      SerialBT.println((String)"Vehicle Battery: " + ext_voltage + "v");

      if (powerstate == 0) {
        Serial.println("PowerState: No Voltage");
        SerialBT.println("PowerState: No Voltage");
      } else if (powerstate == 1) {
        Serial.println("PowerState: Battery Voltage");
        SerialBT.println("PowerState: Battery Voltage");
      } else if (powerstate == 2) {
        Serial.println("PowerState: Alternator Voltage");
        SerialBT.println("PowerState: Alternator Voltage");
      }

      if (Disabled) {
        Serial.println("Vehicle Disabled: ON");
        SerialBT.println("Vehicle Disabled: ON");
      } else {
        Serial.println("Vehicle Disabled: OFF");
        SerialBT.println("Vehicle Disabled: OFF");
      }

      Serial.println((String)"Aquired Satellites: " + sats);
      SerialBT.println((String)"Aquired Satellites: " + sats);

      Serial.print("Latitude: ");
      SerialBT.print("Latitude: ");
      Serial.print(lat, 6);
      SerialBT.print(lat, 6);
      Serial.println("°");
      SerialBT.println("°");

      Serial.print("Longitude: ");
      SerialBT.print("Longitude: ");
      Serial.print(lon, 6);
      SerialBT.print(lon, 6);
      Serial.println("°");
      SerialBT.println("°");

      Serial.println((String)"Distance to Home: " + distance + " miles");
      SerialBT.println((String)"Distance to Home: " + distance + " miles");

      Serial.println((String)"Precision: " + hdop + " meters");
      SerialBT.println((String)"Precision: " + hdop + " meters");

      Serial.println((String)"Speed: " + speed + " mph");
      SerialBT.println((String)"Speed: " + speed + " mph");

      Serial.println((String)"Heading: " + heading + "°");
      SerialBT.println((String)"Heading: " + heading + "°");

      Serial.println((String)"Altitude: " + altitude + " feet");
      SerialBT.println((String)"Altitude: " + altitude + " feet");

      Serial.print("Wifi OTA IP address: ");
      Serial.println(WiFi.localIP());
      SerialBT.print("Wifi OTA IP address: ");
      SerialBT.println(WiFi.localIP());

      Serial.println((String)"Wifi OTA SSID: " + ssid);
      SerialBT.println((String)"Wifi OTA SSID: " + ssid);
      Serial.println((String)"Wifi OTA wpa2key: " + ssidkey);
      SerialBT.println((String)"Wifi OTA wpa2key: " + ssidkey);

      Serial.println((String)"Bluetooth Name: " + BTname);
      SerialBT.println((String)"Bluetooth Name: " + BTname);

      Serial.println((String)"SMS Reply Number: " + smsnumber);
      SerialBT.println((String)"SMS Reply Number: " + smsnumber);

      if (Serial) {
        Serial.println("Serial: Available");
        SerialBT.println("Serial: Available");
      }
      else {
        Serial.println("Serial: Unavailable");
        SerialBT.println("Serial: Unavailable");
      }

      if (SerialBT.hasClient()) {
        Serial.println("Bluetooth Serial: Client Connected");
        SerialBT.println("Bluetooth Serial: Client Connected");
      }
      else {
        Serial.println("Bluetooth Serial: No Connections");
        SerialBT.println("Bluetooth Serial: No Connections");
      }
      Serial.println((String)"Console Timeout: " + ctimeout + " seconds");
      SerialBT.println((String)"Console Timeout: " + ctimeout + " seconds");
      if (gpsfix) {
        Serial.println("GPS Fix: YES");
        SerialBT.println("GPS Fix: YES");
      } else {
        Serial.println("GPS Fix: NO");
        SerialBT.println("GPS Fix: NO");
      }
       if (cellconnected) {
        Serial.println("Cellular Connected: YES");
        SerialBT.println("Cellular Connected: YES");
      } else {
        Serial.println((String)"Cellular Connected: NO");
        SerialBT.println((String)"Cellular Connected: NO");
      }
      break;
    case 100:
      if (debug && unlocked) {
        debug = false;
        EEPROM.write(1, 0);
        EEPROM.commit();
        Serial.println("Debug mode is now: OFF");
        SerialBT.println("Debug mode is now: OFF");
      } else {
        debug = true;
        EEPROM.write(1, 1);
        EEPROM.commit();
        Serial.println("Debug mode is now: ON");
        SerialBT.println("Debug mode is now: ON");
      }
      break;
    case 120:
      disable_vehicle();
      break;
    case 103:
      if (gpsdebug) {
        gpsdebug = false;
        Serial.println("GPS Debug output is now: OFF");
        SerialBT.println("GPS Debug output is now: OFF");
      } else {
        gpsdebug = true;
        Serial.println("GPS Debug output is now: ON");
        SerialBT.println("GPS Debug output is now: ON");
      }
      break;
    default:
      Serial.println("Invalid command");
      SerialBT.println("Invalid command");
      break;
  }
}

void gps_check() {
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
  if (gps.location.isUpdated())
  {
    lat = gps.location.lat();
    lon = gps.location.lng();
    if (gpsdebug) {
      Serial.print((String)"LOCATION Age: " + gps.location.age() + "ms Raw Lat: ");
      Serial.print(gps.location.rawLat().negative ? "-" : "+");
      Serial.print((String)gps.location.rawLat().deg + "[+" + gps.location.rawLat().billionths + " billionths],  Raw Lon: ");
      Serial.print(gps.location.rawLng().negative ? "-" : "+");
      Serial.print((String)gps.location.rawLng().deg + "[+" + gps.location.rawLng().billionths + " billionths],  Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(" Long="));
      Serial.println(gps.location.lng(), 6);
      SerialBT.print((String)"LOCATION Age: " + gps.location.age() + "ms Raw Lat: ");
      SerialBT.print(gps.location.rawLat().negative ? "-" : "+");
      SerialBT.print((String)gps.location.rawLat().deg + "[+" + gps.location.rawLat().billionths + " billionths],  Raw Lon: ");
      SerialBT.print(gps.location.rawLng().negative ? "-" : "+");
      SerialBT.print((String)gps.location.rawLng().deg + "[+" + gps.location.rawLng().billionths + " billionths],  Lat: ");
      SerialBT.print(gps.location.lat(), 6);
      SerialBT.print(F(" Long="));
      SerialBT.println(gps.location.lng(), 6);
    }
  }
  else if (gps.date.isUpdated())
  {
    rtc = RTC.now();
    //if (gps.date.year() != 0 && gps.date.month() != 0) {
      //if (gps.date.year() != rtc.year() || gps.date.month() != rtc.month() || gps.date.day() != rtc.day()) {
        //hwRTC.setYear(gps.date.year() - 2000);
        //hwRTC.setMonth(gps.date.month());
        //hwRTC.setDate(gps.date.day());
        //if (debug && unlocked) {
          //Serial.println((String)"RTC date adjusted to GPS: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " -> " + gps.date.month() + "/" + gps.date.day() + "/" + gps.date.year());
          //SerialBT.println((String)"RTC date adjusted to GPS: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " -> " + gps.date.month() + "/" + gps.date.day() + "/" + gps.date.year());
        //}
      //}
      if (gpsdebug) {
        Serial.println((String)"DATE Fix Age: " + gps.date.age() + "ms Raw: " + gps.date.value() + " Year: " + gps.date.year() + " Month: " + gps.date.month() + " Day=" + gps.date.day());
        SerialBT.println((String)"DATE Fix Age: " + gps.date.age() + "ms Raw: " + gps.date.value() + " Year: " + gps.date.year() + " Month: " + gps.date.month() + " Day=" + gps.date.day());
      }
    }
  //}
  else if (gps.time.isUpdated())
  {
    rtc = RTC.now();
    time_t gpsunix = gpsunixtime();
    time_t rtcunix = rtc.unixtime();
    if (gpsunix != 943920000) {
      if (rtcunix + 10 < gpsunix || rtcunix - 10 > gpsunix) {
        hwRTC.setHour(gps.time.hour());
        hwRTC.setMinute(gps.time.minute());
        hwRTC.setSecond(gps.time.second());
        hwRTC.setYear(gps.date.year() - 2000);
        hwRTC.setMonth(gps.date.month());
        hwRTC.setDate(gps.date.day());
        char* rdt;
        rdt = returndatetime();
        strcpy(last_time_adjust, rdt);
        if (!eeprom.eeprom_write(16, (byte *)last_time_adjust, sizeof(last_time_adjust))) {
          if (debug && unlocked) {
            char buf[50] = "Failed to store data in RTC EEPROM!";
            Serial.println(buf);
            SerialBT.println(buf);
          }
        }

        if (debug && unlocked) {
          Serial.println((String)"RTC time adjusted to GPS: " + rtc.hour() + ":" + rtc.minute() + ":" + rtc.second() + " -> " + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second());
          SerialBT.println((String)"RTC time adjusted to GPS: " + rtc.hour() + ":" + rtc.minute() + ":" + rtc.second() + " -> " + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second());
        }
      }
    }
    if (gpsdebug) {
      Serial.println((String)"TIME Fix Age: " + gps.time.age() + "ms Raw: " + gps.time.value() + " Hour: " + gps.time.hour() + " Minute: " + gps.time.minute() + " Second: " + gps.time.second() + " Hundredths: " + gps.time.centisecond() );
      SerialBT.println((String)"TIME Fix Age: " + gps.time.age() + "ms Raw: " + gps.time.value() + " Hour: " + gps.time.hour() + " Minute: " + gps.time.minute() + " Second: " + gps.time.second() + " Hundredths: " + gps.time.centisecond() );
    }
  }
  else if (gps.speed.isUpdated())
  {
    speed = gps.speed.mph();
    if (gpsdebug) {
      Serial.println((String)"SPEED Fix Age: " + gps.speed.age() + "ms Raw: " + gps.speed.value() + " Knots: " + gps.speed.knots() + " MPH: " + gps.speed.mph() + " m/s: " + gps.speed.mps() + " km/h: " + gps.speed.kmph() );
      SerialBT.println((String)"SPEED Fix Age: " + gps.speed.age() + "ms Raw: " + gps.speed.value() + " Knots: " + gps.speed.knots() + " MPH: " + gps.speed.mph() + " m/s: " + gps.speed.mps() + " km/h: " + gps.speed.kmph() );
    }
  }
  else if (gps.course.isUpdated())
  {
    heading = gps.course.deg();
    if (gpsdebug) {
      Serial.println((String)"COURSE Fix Age: " + gps.course.age() + "ms Raw: " + gps.course.value() + " Deg: " + gps.course.deg() );
      SerialBT.println((String)"COURSE Fix Age: " + gps.course.age() + "ms Raw: " + gps.course.value() + " Deg: " + gps.course.deg() );
    }
  }
  else if (gps.altitude.isUpdated())
  {
    altitude = gps.altitude.feet();
    if (gpsdebug) {
      Serial.print((String)"ALTITUDE Fix Age: " + gps.altitude.age() + "ms Raw: " + gps.altitude.value() + " Meters: " + gps.altitude.meters() + " Miles: " + gps.altitude.miles() + " KM: " + gps.altitude.kilometers() + " Feet: " + gps.altitude.feet());
      SerialBT.print((String)"ALTITUDE Fix Age: " + gps.altitude.age() + "ms Raw: " + gps.altitude.value() + " Meters: " + gps.altitude.meters() + " Miles: " + gps.altitude.miles() + " KM: " + gps.altitude.kilometers() + " Feet: " + gps.altitude.feet());
    }
  }
  else if (gps.satellites.isUpdated())
  {
    sats = gps.satellites.value();
    if (sats > 0 && gpsfix == false) {
      gpsfix = true;
      if (debug && unlocked) {
        Serial.println((String)"GPS fix aquired with "+sats+" satellites");
        SerialBT.println((String)"GPS fix aquired with "+sats+" satellites");
      }
    } else if (sats == 0) {
      gpsfix = false;
    }
    if (gpsdebug) {
      Serial.println((String)"SATELLITES Fix Age: " + gps.satellites.age() + "ms Value: " + gps.satellites.value());
      SerialBT.println((String)"SATELLITES Fix Age: " + gps.satellites.age() + "ms Value: " + gps.satellites.value());
    }
  }
  else if (gps.hdop.isUpdated())
  {
    hdop = gps.hdop.hdop();
    if (gpsdebug) {
      Serial.println((String)"HDOP Fix Age: " + gps.hdop.age() + "ms raw: " + gps.hdop.value() + " hdop: " + gps.hdop.hdop());
      SerialBT.println((String)"HDOP Fix Age: " + gps.hdop.age() + "ms raw: " + gps.hdop.value() + " hdop: " + gps.hdop.hdop());
    }
  }
  else if (millis() - last > 5000)
  {
    if (gps.location.isValid())
    {
      static const double HOME_LAT = 42.549379, HOME_LON = -82.923508;
      double distanceToHome =
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
      distance = (distanceToHome / 1000) * 0.62137;
      if (gpsdebug) {
        Serial.println((String)"HOME Distance: " + distance + " mi Course-to=" + courseToHome + " degrees [" + TinyGPSPlus::cardinal(courseToHome) + "]");
        SerialBT.println((String)"HOME Distance: " + distance + " mi Course-to=" + courseToHome + " degrees [" + TinyGPSPlus::cardinal(courseToHome) + "]");
      }
    }
    if (gpsdebug) {
      Serial.println((String)"DIAGS Chars: " + gps.charsProcessed() + " Sentences-with-Fix: " + gps.sentencesWithFix() + " Failed-checksum: " + gps.failedChecksum() + " Passed-checksum: " + gps.passedChecksum());
      SerialBT.println((String)"DIAGS Chars: " + gps.charsProcessed() + " Sentences-with-Fix: " + gps.sentencesWithFix() + " Failed-checksum: " + gps.failedChecksum() + " Passed-checksum: " + gps.passedChecksum());
    }
    if (gps.charsProcessed() < 10 && gpsdebug) {
      Serial.println(F("WARNING: No GPS data.  Check wiring."));
      SerialBT.println(F("WARNING: No GPS data.  Check wiring."));
    }
    last = millis();
  }
}

void overtemp_check() {
  if (temp > OVERTEMP) {
    if (debug && unlocked) {
      Serial.println((String)temp + " > " + OVERTEMP + " Over Temperature Shutdown for 10 min");
      SerialBT.println((String)temp + " > " + OVERTEMP + " Over Temperature Shutdown for 10 min");
    }
    TIME_TO_SLEEP = 600;
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    if (debug && unlocked) {
      Serial.flush();
      SerialBT.flush();
    }
    esp_deep_sleep_start();
  } else if (temp > OVERTEMP) {
    if (debug && unlocked) {
      Serial.println((String)temp + " < " + UNDERTEMP + " Under Temperature Shutdown for 10 min");
      SerialBT.println((String)temp + " < " + UNDERTEMP + " Under Temperature Shutdown for 10 min");
    }
    TIME_TO_SLEEP = 600;
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    if (debug && unlocked) {
      Serial.flush();
      SerialBT.flush();
    }
    esp_deep_sleep_start();
  }
}

void deepsleep() {
  if (!updating && !Disabled) {
    //digitalWrite(GPS_POWER_ON, LOW);  // Power off GPS module
    if (powerstate == 0) {
      if (int_voltage < 3.3) {
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        TIME_TO_SLEEP = 31536000;
      char* rdt;
      rdt = returndatetime();
      strcpy(last_sleep, rdt);
        strcat(last_sleep, " | Low Int Batt");
              Serial.println(last_sleep);
      Serial.println(sizeof(last_sleep));
        eeprom.eeprom_write(255, (byte *)last_sleep, sizeof(last_sleep));  // Write last sleep time & reason
        if (debug && unlocked) {
          Serial.println("Going to sleep forever due to low internal battery...");
          SerialBT.println("Going to sleep forever due to low internal battery...");
        }
      } else {
       char* rdt;
      rdt = returndatetime();
      strcpy(last_sleep, rdt);
        strcat(last_sleep, " | Low Car Batt");
              Serial.println(last_sleep);
      Serial.println(sizeof(last_sleep));
        eeprom.eeprom_write(255, (byte *)last_sleep, sizeof(last_sleep));  // Write last sleep time & reason       
        if (debug && unlocked) {
          Serial.println("Going to sleep due to low vehicle battery...");
          SerialBT.println("Going to sleep due to low vehicle battery...");
        }
        TIME_TO_SLEEP = 3600;
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
      }
    } else if (powerstate == 1) {
      
      char* rdt;
      rdt = returndatetime();
      strcpy(last_sleep, rdt);
      strcat(last_sleep, " | Vehicle Shutoff");
      Serial.println(last_sleep);
      Serial.println(sizeof(last_sleep));
      eeprom.eeprom_write(255, (byte *)last_sleep, sizeof(last_sleep));  // Write last sleep time & reason
      if (debug && unlocked) {
        Serial.println("Going to sleep due to vehicle shutoff...");
        SerialBT.println("Going to sleep due to vehicle shutoff...");
      }
      TIME_TO_SLEEP = 3600;
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
    }
    if (debug && unlocked) {
      Serial.flush();
      SerialBT.flush();
    }
    esp_deep_sleep_start();
  } else {
    if (debug && unlocked) {
      Serial.println("Aborting sleep due to updating or vehicle disabled");
      SerialBT.println("Aborting sleep due to updating console or vehicle disabled");
    }
  }
}

void print_wakeup_reason() {
  if (debug && unlocked) {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
      case ESP_SLEEP_WAKEUP_EXT0: {
          Serial.println("Wakeup caused by external signal using RTC_IO");
          SerialBT.println("Wakeup caused by external signal using RTC_IO");
          break;
        }
      case ESP_SLEEP_WAKEUP_EXT1: {
          Serial.println("Wakeup caused by external signal using RTC_CNTL");
          SerialBT.println("Wakeup caused by external signal using RTC_CNTL");
          break;
        }
      case ESP_SLEEP_WAKEUP_TIMER: {
          Serial.println("Wakeup caused by timer");
          SerialBT.println("Wakeup caused by timer");
          break;
        }
      case ESP_SLEEP_WAKEUP_TOUCHPAD: {
          Serial.println("Wakeup caused by touchpad");
          SerialBT.println("Wakeup caused by touchpad");
          break;
        }
      case ESP_SLEEP_WAKEUP_ULP: {
          Serial.println("Wakeup caused by ULP program");
          SerialBT.println("Wakeup caused by ULP program");
          break;
        }
      default: {
          Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
          SerialBT.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
          break;
        }
    }
  }
}

void disable_vehicle() {
  if (!Disabled) {
    Disabled = true;
    EEPROM.write(0, 1);
    EEPROM.commit();
    digitalWrite(RELAY_PIN, HIGH);
    if (debug && unlocked) {
      Serial.println("Vehicle is now DISABLED!");
      SerialBT.println("Vehicle is now DISABLED!");
    }
    char* smsmsg;
    char* db;
    char* db2;
    dtostrf(lat, 8, 4, db);
    dtostrf(lon, 8, 4, db2);
    //String smsmsg = "Vehicle Disabled\n https://maps.google.com/?q="+db+","+db2;
    Serial.println(db);
    if (RESPOND == 1) {
      int smserror = GSM.sendSms(smsnumber, smsmsg);
      if (debug && unlocked) {
        Serial.println((String)"Sending SMS error code: " + smserror);
        SerialBT.println((String)"Sending SMS error code: " + smserror);
      }
    }
  } else if (Disabled) {
    Disabled = false;
    EEPROM.write(0, 0);
    EEPROM.commit();
    digitalWrite(RELAY_PIN, LOW);
    if (debug && unlocked) {
      Serial.println("Vehicle has been RE-ENABLED");
      SerialBT.println("Vehicle has been RE-ENABLED");
    }
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
  ext_voltage = vOut * factor;
  if (ext_voltage < 10) {
    powerstate = 0;
  } else if (ext_voltage <= 12.8) {
    powerstate = 1;
  } else if (ext_voltage > 12.8) {
    powerstate = 2;
  }
}

void print_status() {
  if (debug && unlocked) {
    rtc = RTC.now();
    Serial.print((String)"RTC: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " " + rtc.hour() + ":" + rtc.minute() + " | Temp: " + temp + "°C | Ext Batt: " + ext_voltage + "v | Int Batt: " + int_voltage + "v (" + batteryLevel + "%) | " + "PowerState: ");
    SerialBT.print((String)"RTC: " + rtc.month() + "/" + rtc.day() + "/" + rtc.year() + " " + rtc.hour() + ":" + rtc.minute() + " | Temp: " + temp + "°C | Ext Batt: " + ext_voltage + "v | Int Batt: " + int_voltage + "v (" + batteryLevel + "%) | " + "PowerState: ");
    if (ext_voltage < 10) {
      powerstate = 0;
      Serial.print("No Voltage");
      SerialBT.print("No Voltage");
    } else if (ext_voltage <= 12.8) {
      powerstate = 1;
      Serial.print("Battery Voltage");
      SerialBT.print("Battery Voltage");
    } else if (ext_voltage > 12.8) {
      powerstate = 2;
      Serial.print("Alternator Voltage");
      SerialBT.print("Alternator Voltage");
    }
    if (Disabled) {
      Serial.println(" | Vehicle Disabled: ON");
      SerialBT.println(" | Vehicle Disabled: ON");
    } else {
      Serial.println(" | Vehicle Disabled: OFF");
      SerialBT.println(" | Vehicle Disabled: OFF");
    }
  }
}

void reset_device() {
  if (debug && unlocked) {
    Serial.println("Resetting Device...");
    SerialBT.println("Resetting Device...");
    Serial.flush();
    SerialBT.flush();
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
  int_voltage = ((((raw/4095)*2)*3.3)*1100)/1000;
  batteryLevel = _min(map(int_voltage, 3.3, 4.2, 0, 100), 100);
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
  int_voltage = (sum / INT_SAMPLES) / 2350.0;
  batteryLevel = _min(map(sum / INT_SAMPLES, 2000, 2350.0, 0, 100), 100);  //1100
  if (sum / INT_SAMPLES < 1200) batteryLevel = 0;
  sample_count = 0;
  sum = 0;
  int_voltage = int_voltage * 4.20;
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
  if (debug && unlocked) {
    Serial.println(buf);
    SerialBT.println(buf);
  }
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
