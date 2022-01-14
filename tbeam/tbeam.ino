//All pin definitions for T-BEAM V10 (rev 1)
#include <Wire.h>
#include "I2C_AXP192.h"

#include <TinyGPS++.h>

#define I2C_SDA    21
#define I2C_SCL   22
#define GPS_RX_PIN  34
#define GPS_TX_PIN  12
#define LED_PIN         14
#define BUTTON_PIN      38

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#define RESET_GPIO      14
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire);

/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 34, TXPin = 12;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
  Wire.begin(21, 22);
  delay(100);

  I2C_AXP192_InitDef initDef = {
    .EXTEN  = true,
    .BACKUP = true,
    .DCDC1  = 3300,
    .DCDC2  = 0,
    .DCDC3  = 0,
    .LDO2   = 3000,
    .LDO3   = 3000,
    .GPIO0  = 2800,
    .GPIO1  = -1,
    .GPIO2  = -1,
    .GPIO3  = -1,
    .GPIO4  = -1,
  };
  axp192.begin(initDef);

  pinMode(BUTTON_PIN, INPUT);

  Serial.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  Serial.printf("getBatteryVoltage()          : %7.2f\n", axp192.getBatteryVoltage());
  Serial.printf("getBatteryDischargeCurrent() : %7.2f\n", axp192.getBatteryDischargeCurrent());
  Serial.printf("getBatteryChargeCurrent()    : %7.2f\n", axp192.getBatteryChargeCurrent());
  Serial.printf("getAcinVolatge()             : %7.2f\n", axp192.getAcinVolatge());
  Serial.printf("getAcinCurrent()             : %7.2f\n", axp192.getAcinCurrent());
  Serial.printf("getVbusVoltage()             : %7.2f\n", axp192.getVbusVoltage());
  Serial.printf("getVbusCurrent()             : %7.2f\n", axp192.getVbusCurrent());
  Serial.printf("getInternalTemperature()     : %7.2f\n", axp192.getInternalTemperature());
  Serial.printf("getApsVoltage()              : %7.2f\n", axp192.getApsVoltage());
  Serial.printf("getPekPress()                : %4d\n"  , axp192.getPekPress());

  Serial.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
