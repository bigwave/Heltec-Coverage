/**
 * This is an example of joining, sending and receiving data via LoRaWAN using a more minimal interface.
 * 
 * The example is configured for OTAA, set your keys into the variables below.
 * 
 * The example will upload a counter value periodically, and will print any downlink messages.
 * 
 * David Brodrick.
 */
#include "LoRaWanMinimal_APP.h"
#include "loramac/system/timeServer.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include <Wire.h>
#include "cubecell_SSD1306Wire.h"
#include <TimeLib.h>
#include "CubeCell_NeoPixel.h"

CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);

SSD1306Wire display(0x3c, 500000, I2C_NUM_0, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst

//when gps waked, if in GPS_UPDATE_TIMEOUT, gps not fixed then into low power mode
#define GPS_UPDATE_TIMEOUT 120000

//once fixed, GPS_CONTINUE_TIME later into low power mode
#define GPS_CONTINUE_TIME 10000
/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

//Set these OTAA parameters to match your app/node in TTN
/* OTAA para*/
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0xD2, 0x05, 0x9E, 0xDD, 0x99, 0x16, 0x92, 0xB9, 0x4D, 0x82, 0x62, 0xC1, 0xC9, 0x28, 0xA0, 0xC9};

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

static uint32_t lastGpsAge = UINT32_MAX;
static time_t lastGpsFix;
static time_t lastLoRaWanAck;
static uint8_t lastRssi;

static uint16 redLed = 0;
static uint16 greenLed = 0;
static uint16 blueLed = 0;

uint8_t appDataSize = 0;
uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

static void wakeUp()
{
  sleepTimerExpired = true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  displayDateTime();
  displayInfo();
  sleepTimerExpired = false;
  TimerInit(&sleepTimer, &wakeUp);
  TimerSetValue(&sleepTimer, sleeptime);
  TimerStart(&sleepTimer);
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired)
    lowPowerHandler();
  TimerStop(&sleepTimer);
}

int32_t fracPart(double val, int n)
{
  return (int32_t)((val - (int32_t)(val)) * pow(10, n));
}
void displayInfo()
{
  Serial.print("Date/Time: ");
  if (Air530.date.isValid())
  {
    Serial.printf("%d/%02d/%02d", Air530.date.year(), Air530.date.day(), Air530.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (Air530.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second(), Air530.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();

  Serial.print("LAT: ");
  Serial.print(Air530.location.lat(), 6);
  Serial.print(", LON: ");
  Serial.print(Air530.location.lng(), 6);
  Serial.print(", ALT: ");
  Serial.print(Air530.altitude.meters());

  Serial.println();

  Serial.print("SATS: ");
  Serial.print(Air530.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(Air530.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(Air530.location.age());
  Serial.print(", COURSE: ");
  Serial.print(Air530.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(Air530.speed.kmph());
  Serial.println();
}

void displayGPSInfo()
{
  char str[30];

  // if ( Air530.location.age() < 1000 )
  // {
  //   display.drawString(120, 0, "A");
  // }
  // else
  // {
  //   display.drawString(120, 0, "V");
  // }

  int index = sprintf(str, "alt: %d.%d", (int)Air530.altitude.meters(), fracPart(Air530.altitude.meters(), 2));
  str[index] = 0;
  display.drawString(70, 42, str);

  index = sprintf(str, "hdop: %d.%d", (int)Air530.hdop.hdop(), fracPart(Air530.hdop.hdop(), 2));
  str[index] = 0;
  display.drawString(70, 54, str);

  index = sprintf(str, "lat : %d.%d", (int)Air530.location.lat(), fracPart(Air530.location.lat(), 4));
  str[index] = 0;
  display.drawString(0, 42, str);

  index = sprintf(str, "lon : %d.%d", (int)Air530.location.lng(), fracPart(Air530.location.lng(), 4));
  str[index] = 0;
  display.drawString(0, 54, str);

  // index = sprintf(str, "speed: %d.%d km/h", (int)Air530.speed.kmph(), fracPart(Air530.speed.kmph(), 3));
  // str[index] = 0;
  // display.drawString(0, 48, str);
  display.display();
}
void displayDateTime()
{
  display.setColor(BLACK);
  display.fillRect(0, 0, 128, 40);
  display.setColor(WHITE);
  char str[30];

  int index = sprintf(str, "%02d-%02d-%02d", year(), day(), month());
  str[index] = 0;
  display.drawString(0, 0, str);
  index = sprintf(str, "%02d:%02d:%02d", hour(), minute(), second());
  str[index] = 0;
  display.drawString(60, 0, str);

  // Serial.print ("Raw voltage : ");
  // Serial.println(getBatteryVoltage());
  double batteryVoltage = getBatteryVoltage() / 1000.0;
  // Serial.print ("double voltage : ");
  // Serial.println(batteryVoltage);
  // Serial.print ("fracPart(batteryVoltage, 1)  : ");
  // Serial.println(fracPart(batteryVoltage, 1));
  index = sprintf(str, "%d.%dV", (int)batteryVoltage, fracPart(batteryVoltage, 1));
  str[index] = 0;
  display.drawString(105, 0, str);

  display.drawString(0, 10, "Last GPS:");
  index = sprintf(str, "-%ds", now() - lastGpsFix);
  str[index] = 0;
  display.drawString(60, 10, str);

  display.drawString(0, 20, "Last ACK:");
  index = sprintf(str, "-%ds", now() - lastLoRaWanAck);
  str[index] = 0;
  display.drawString(60, 20, str);

  display.drawString(0, 30, "Last RSSI:");
  index = sprintf(str, "-%d dBm", lastRssi);
  str[index] = 0;
  display.drawString(60, 30, str);

  display.display();
}

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

static void prepareTxFrame()
{
  /*appData size is LoRaWan_APP_Custom_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LoRaWan_APP_Custom_DATA_MAX_SIZE.
    if enabled AT, don't modify LoRaWan_APP_Custom_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LoRaWan_APP_Custom_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  float lat, lon, alt, course, speed, hdop, sats;

  lat = Air530.location.lat();
  lon = Air530.location.lng();
  alt = Air530.altitude.meters();
  course = Air530.course.deg();
  speed = Air530.speed.kmph();
  sats = Air530.satellites.value();
  hdop = Air530.hdop.hdop();

  uint16_t batteryVoltage = getBatteryVoltage();

  unsigned char *puc;

  appDataSize = 0;
  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&alt);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&course);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&speed);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&hdop);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  appData[appDataSize++] = (uint8_t)(batteryVoltage >> 8);
  appData[appDataSize++] = (uint8_t)batteryVoltage;
}

///////////////////////////////////////////////////
void setup()
{
  boardInitMcu();
  Serial.begin(115200);
  uint64_t chipID = getID();
  Serial.printf("ChipID:%04X%08X\r\n", (uint32_t)(chipID >> 32), (uint32_t)chipID);

  VextON();
  delay(10);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); //SET POWER
  delay(1);
  pixels.begin(); // INITIALIZE RGB strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setBrightness(4);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  display.init();
  display.clear();
  display.setBrightness(64);
  display.display();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 32 - 16 / 2, "GPS initing...");
  display.display();

  Air530.begin();
  Air530.setmode(MODE_GPS_GLONASS);
  Air530.setPPS(3, 500);

  if (ACTIVE_REGION == LORAMAC_REGION_AU915)
  {
    //TTN uses sub-band 2 in AU915
    LoRaWAN.setSubBand2();
  }

  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);

  //Enable ADR
  LoRaWAN.setAdaptiveDR(true);
  display.clear();
  display.display();

  display.drawString(0, 0, "LORAWAN Joining");
  display.display();

  while (1)
  {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey);
    if (!LoRaWAN.isJoined())
    {
      //In this example we just loop until we're joined, but you could
      //also go and start doing other things and try again later
      display.clear();
      display.display();

      display.drawString(0, 0, "LORAWAN JOIN FAILED! Sleeping for 30 seconds...");
      display.display();
      Serial.println("JOIN FAILED! Sleeping for 30 seconds");
      lowPowerSleep(30000);
    }
    else
    {
      display.clear();
      display.display();

      display.drawString(0, 0, "LORAWAN JOINED...");
      display.display();
      Serial.println("JOINED");
      lowPowerSleep(500);
      break;
    }
  }
}
//A 'do nothing' function for timer callbacks
static void wakeUpDummy() {}

void displayRgb()
{

  if (lastLoRaWanAck == 0)
  {
    return;
  }

  time_t test = now();
  uint32_t redLed = (test - lastLoRaWanAck) / 10;
  if (redLed > 255)
  {
    redLed = 255;
  }

  uint32_t greenLed = lastRssi;

  uint32_t blueLed = 0;
  if (now() - lastLoRaWanAck == 1)
  {
    blueLed = 255;
  }
  pixels.setPixelColor(0, pixels.Color(redLed, 0, 0));
  //pixels.setPixelColor(0, pixels.Color(redLed, greenLed, blueLed));
  pixels.show(); // Send the updated pixel colors to the hardware.
}
///////////////////////////////////////////////////
void loop()
{
  displayDateTime();
  displayRgb();
  if (LoRaWAN.busy())
  {
    TimerEvent_t pollStateTimer;
    TimerInit(&pollStateTimer, wakeUpDummy);
    TimerSetValue(&pollStateTimer, 100);
    //Serial.println("LORAWAN BUSY");
    TimerStart(&pollStateTimer);
    lowPowerHandler();
    TimerStop(&pollStateTimer);
    Radio.IrqProcess();
    return;
  }

  uint32_t starttime = millis();
  while ((millis() - starttime) < 1000)
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
  }

  if (!Air530.location.isValid())
  {
    return;
  }
  if (Air530.hdop.hdop() > 5)
  {
    return;
  }
  setTime(Air530.time.hour(),
          Air530.time.minute(),
          Air530.time.second(),
          Air530.date.day(),
          Air530.date.month(),
          Air530.date.year());
  uint32_t currentAge = Air530.location.age();
  // Serial.print("Previous AGE: ");
  // Serial.println(lastGpsAge);
  // Serial.print("current  AGE: ");
  // Serial.println(currentAge);

  if (currentAge < lastGpsAge)
  {
    lastGpsFix = now();
    display.clear();
    displayGPSInfo();
    displayDateTime();
    displayInfo();
    prepareTxFrame();
    if (LoRaWAN.send(appDataSize, appData, 2, true))
    {
      Serial.println("Send OK");
    }
    else
    {
      Serial.println("Send FAILED");
    }

    lastGpsAge = Air530.location.age();
  }
  
  DeviceClass_t lorawanClass = LORAWAN_CLASS;
  if (lorawanClass == CLASS_A)
  { 
          Serial.println("lowPowerSleep");

    lowPowerSleep(15000);
  }
}

//Counter is just some dummy data we send for the example
//counter++;

//In this demo we use a timer to go into low power mode to kill some time.
//You might be collecting data or doing something more interesting instead.
//

//Now send the data. The parameters are "data size, data pointer, port, request ack"
// Serial.printf("\nSending packet with counter=%d\n", counter);
//Here we send confirmed packed (ACK requested) only for the first five (remember there is a fair use policy)
// bool requestack=counter<5?true:false;
// if (LoRaWAN.send(1, &counter, 1, requestack)) {
//   Serial.println("Send OK");
// } else {
//   Serial.println("Send FAILED");
// }

///////////////////////////////////////////////////
//Example of handling downlink data
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Received downlink: %s, RXSIZE %d, PORT %d, DATA: ", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
  {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();
}

void myLoRaWanFCNCheck(bool ackReceived, uint8_t rssi)
{
  Serial.println("ACK RECEIVED");
  if (ackReceived)
  {
    lastLoRaWanAck = now();
  }
  lastRssi = rssi;
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}