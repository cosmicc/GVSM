  /*
  ESP32 EEPROM MAP (4k):
  0: Vehicle disable byte
  1: Debug byte

  RTC EEPROM MAP (32k):
  16-48: Last time adjust (32 Bytes)
  50-98: Last restart 1 timestamp/reason (48 Bytes)
  100-148: Last restart 2 timestamp & Reason (48 Bytes)
  150-198: Last restart 3 timestamp & Reason (48 Bytes)
  200-206: Wifi SSID (7 Bytes)
  208-224: Wifi Key (17 Bytes)
  225-232: Console Password
  233-240: Bluetooth Discovery Name
  241-252: SMS Phone Number
  253-254: Console Timeout Seconds (2 Bytes)
  255-303: Last recorded sleep time & reason (48 Bytes)

  500-509: Last Lattitude
  510-519: Last Longitude
  520-525: Last Distance from home
 

#include <Wire.h>
#include <uEEPROMLib.h>

char BTname[8] = "ESP32";            // Bluetooth Discovery Name
char ssid[7] = "mySSID";             // Wifi SSID for OTA updates (Stored in RTC EEPROM)
char ssidkey[17] = "myKEY";          // Wifi SSID key for OTA updates (Stored in RTC EEPROM)
char consolepw[8] = "password";      // Console Password (Stored in RTC EEPROM)
char smsnumber[12] = "+15555551212"; // SMS reply number
short ctimeout[2] = 60;                // Console timeout seconds

void setup() {
  Serial.begin(115200);
  uEEPROMLib eeprom(0x57);
   
  eeprom.eeprom_write(200, (byte *)ssid, 6);  // Read SSID
  eeprom.eeprom_write(208, (byte *)ssidkey, 16);  // Read SSID key
  eeprom.eeprom_write(225, (byte *)consolepw, 8);  // Read Console password
  eeprom.eeprom_write(233, (byte *)BTname, 8);  // Read Bluetooth name
  eeprom.eeprom_write(241, (byte *)smsnumber, 12);  // Read SMS reply number
  eeprom.eeprom_write(253, ctimeout);  // Read Console Timeout
  Serial.println("EEPROM write complete");
}

void loop() {
  // nothing to do here
}
*/
