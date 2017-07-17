#include <ESP8266WiFi.h>
//#include <AsyncEventSource.h>
//#include <AsyncJson.h>
//#include <AsyncWebSocket.h>
#include <ESPAsyncWebServer.h>
//#include <SPIFFSEditor.h>
//#include <StringArray.h>
//#include <WebAuthentication.h>
//#include <WebHandlerImpl.h>
//#include <WebResponseImpl.h>
#include <ESP8266HTTPClient.h>
#include <RingBuf.h>
#include "SettingsFile.h"
/*
 *  Contains code from: 
 *  http://randomnerdtutorials.com/esp8266-remote-controlled-sockets/
 *  http://www.bruhautomation.com/p/cheapest-wifi-automated-blinds.html
 *  https://www.arduino.cc/en/Tutorial/Sweep
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */

// in Settingsfile.h
//#define WLAN_SSID "xxx"
//#define WLAN_PASS "xxx"
//#define ws_Port 8151
//IPAddress stHubIP(192, 168, x, x);

//Enable/disable functions
#define Enable_Wifi 1

//Acurite stuff
#define RING_BUFFER_SIZE  152

#define SYNC_HIGH 600
#define SYNC_LOW 600

#define PULSE_LONG 400
#define PULSE_SHORT 220

#define PULSE_RANGE 100

//BIT1
#define BIT1_HIGH_MIN (PULSE_LONG - PULSE_RANGE)
#define BIT1_HIGH_MAX (PULSE_LONG + PULSE_RANGE)
#define BIT1_LOW_MIN (PULSE_SHORT - PULSE_RANGE)
#define BIT1_LOW_MAX (PULSE_SHORT + PULSE_RANGE)

//BIT0
#define BIT0_HIGH_MIN (PULSE_SHORT - PULSE_RANGE)
#define BIT0_HIGH_MAX (PULSE_SHORT + PULSE_RANGE)
#define BIT0_LOW_MIN (PULSE_LONG - PULSE_RANGE)
#define BIT0_LOW_MAX (PULSE_LONG + PULSE_RANGE)

#define DATAPIN (3) // D3 is interrupt 1
#define LAPIN (5)   //logic Analyzer PIN
#define SQUELCHPIN (4)
#define SYNCPULSECNT (4) // 4 pulses (8 edges)
#define SYNCPULSEEDGES (SYNCPULSECNT * 2)

#define DATABYTESCNT_MIN (7)                   // Minimum number of data bytes
#define DATABITSCNT_MIN (DATABYTESCNT_MIN * 8) // 7 bytes * 8 bits
#define DATABITSEDGES_MIN (DATABITSCNT_MIN * 2)

#define DATABYTESCNT_MID 128 //8 Bytes

#define DATABYTESCNT_MAX (9)                   // 9 Bytes
#define DATABITSCNT_MAX (DATABYTESCNT_MAX * 8) // 7 bytes * 8 bits
#define DATABITSEDGES_MAX (DATABITSCNT_MAX * 2)

// 5n1 Tower Message Types
#define MT_WS_WD_RF 49 // wind speed, wind direction, rainfall
#define MT_WS_T_RH 56  // wind speed, temp, RH

#define eventTimeoutms 7200000 //Timeout in miliseconds before an event is over

// macros from DateTime.h
/* Useful Constants */
#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) (_time_ / SECS_PER_DAY)

// The pulse durations are the measured time in micro seconds between
// pulse edges.
unsigned long pulseDurations[RING_BUFFER_SIZE];
unsigned int syncIndex = 0; // index of the last bit time of the sync signal
unsigned int dataIndex = 0; // index of the first bit time of the data bits (syncIndex+1)
bool syncFound = false;     // true if sync pulses found
bool received = false;      // true if sync plus enough bits found
unsigned int changeCount = 0;
unsigned int bytesReceived = 0;

int acurite_5n1_raincounter = 0;
int rainWrapOffset = 0;
int lastRainCount = 0;
bool activeRain = false;
unsigned long rainLast = 0;

int strikeTot = 0;
int strikeWrapOffset = 0;
int lastStrikeCount = 0;
bool activeStrikes = false;
unsigned long strikeLast = 0;

const unsigned int tempOffset10th = 0; // offset in 10th degrees C

const float winddirections[] = {315.0, 247.5, 292.5, 270.0,
                                337.5, 225.0, 0.0, 202.5,
                                67.5, 135.0, 90.0, 112.5,
                                45.0, 157.5, 22.5, 180.0};

char *acurite_5n1_winddirection_str[] =
    {"NW",  // 0  315
     "WSW", // 1  247.5
     "WNW", // 2  292.5
     "W",   // 3  270
     "NNW", // 4  337.5
     "SW",  // 5  225
     "N",   // 6  0
     "SSW", // 7  202.5
     "ENE", // 8  67.5
     "SE",  // 9  135
     "E",   // 10 90
     "ESE", // 11 112.5
     "NE",  // 12 45
     "SSE", // 13 157.5
     "NNE", // 14 22.5
     "S"};  // 15 180

//Webserver Stuff

AsyncWebServer server(ws_Port);

int stPort = 80;
HTTPClient hClient;

RingBuf *resBuff = RingBuf_new(80, 20);

/*
 * Look for the sync pulse train, 4 high-low pulses of
 * 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses
 * approximately 600 uS long.
 */
bool isSync(unsigned int idx)
{
  // check if we've received 4 pulses of matching timing
  for (int i = 0; i < SYNCPULSEEDGES; i += 2)
  {
    unsigned long t1 = pulseDurations[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
    unsigned long t0 = pulseDurations[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];

    // any of the preceeding 8 pulses are out of bounds, short or long,
    // return false, no sync found
    if (t0 < (SYNC_HIGH - PULSE_RANGE) || t0 > (SYNC_HIGH + PULSE_RANGE) ||
        t1 < (SYNC_LOW - PULSE_RANGE) || t1 > (SYNC_LOW + PULSE_RANGE))
    {
      return false;
    }
  }
  return true;
}

/* Interrupt 1 handler 
 * Tied to pin 3 INT1 of arduino.
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the Arduino LED (pin 13) on each interrupt. 
 * This allows scoping pin 13 to see the interrupt / data pulse train.
 */
void handler()
{
  static unsigned long duration = 0;
  static unsigned long lastTime = 0;
  static unsigned int ringIndex = 0;
  static unsigned int syncCount = 0;
  static unsigned int bitState = 0;

  bitState = digitalRead(DATAPIN);
  digitalWrite(13, bitState);

  // ignore if we haven't finished processing the previous
  // received signal in the main loop.
  if (received == true)
  {
    return;
  }

  // calculating timing since last change
  long time = micros();
  duration = time - lastTime;
  lastTime = time;

  // Known error in bit stream is runt/short pulses.
  // If we ever get a really short, or really long,
  // pulse we know there is an error in the bit stream
  // and should start over.
  if ((duration > (PULSE_LONG + PULSE_RANGE)) || (duration < (PULSE_SHORT - PULSE_RANGE)))
  {
    //Check to see if we have received a minimum number of bits we could take

    if (syncFound && changeCount >= DATABITSEDGES_MIN)
    {
      if (changeCount >= DATABYTESCNT_MID)
      {
        bytesReceived = 8;
      }
      else
      {
        bytesReceived = 7;
      }
      received = true;
      return;
    }
    else
    {
      received = false;
      syncFound = false;
      changeCount = 0; // restart looking for data bits
    }

    digitalWrite(LAPIN, LOW);
  }

  // store data in ring buffer
  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  pulseDurations[ringIndex] = duration;
  changeCount++; // found another edge

  // detect sync signal
  if (isSync(ringIndex))
  {
    syncFound = true;
    changeCount = 0; // restart looking for data bits
    syncIndex = ringIndex;
    dataIndex = (syncIndex + 1) % RING_BUFFER_SIZE;
    digitalWrite(LAPIN, HIGH);
  }

  // If a sync has been found the start looking for the
  // DATABITSEDGES data bit edges.
  if (syncFound)
  {
    // if not enough bits yet, no message received yet
    if (changeCount < DATABITSEDGES_MAX)
    {

      received = false;
    }
    else if (changeCount > DATABITSEDGES_MAX)
    {
      // if too many bits received then reset and start over
      received = false;
      syncFound = false;
      digitalWrite(LAPIN, LOW);
    }
    else
    {
      received = true;
      bytesReceived = 9;
      digitalWrite(LAPIN, LOW);
    }
  }
}

void setup(void)
{

  Serial.begin(115200);
  Serial.println("Setup Sequence\n");
  pinMode(DATAPIN, INPUT); // data interrupt input
  pinMode(13, OUTPUT);     // LED output
  pinMode(LAPIN, OUTPUT);
  digitalWrite(LAPIN, LOW);
  attachInterrupt(1, handler, CHANGE);
  pinMode(SQUELCHPIN, OUTPUT);    // data squelch pin on radio module if you have hone
  digitalWrite(SQUELCHPIN, HIGH); // UN-squelch data

#ifdef Enable_Wifi
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  //WiFi.begin(WLAN_SSID, WLAN_PASS);
  WiFi.begin(WLAN_SSID);
  delay(500);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Listening on Port:");
  Serial.println(ws_Port);

  server.on("/refresh", HTTP_ANY, [](AsyncWebServerRequest *request) {
    char *respData;
    Serial.println("Received Refresh");
    Serial.print("Arguments Received:");
    int params = request->params();

    Serial.println(params);

    // for (int i = 0; i < params; i++)
    // {
    //   AsyncWebParameter *p = request->getParam(i);
    //   if (p->isFile())
    //   {
    //     Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
    //   }
    //   else if (p->isPost())
    //   {
    //     Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    //   }
    //   else
    //   {
    //     Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
    //   }
    // }

    if (resBuff->isEmpty(resBuff))
    {
      respData = (char *)"mo=0";
    }
    else
    {
      resBuff->pull(resBuff, &respData);
      if (!resBuff->isEmpty(resBuff))
      {
        strcat(respData, "&mo=1");
      }
    }

    Serial.print("Size of Response - ");
    Serial.print(sizeof(respData));
    Serial.printf("%s \n", respData);

    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", respData);
    response->addHeader("Server", "ESP Async Web Server");
    request->send(response);

    Serial.println("Transmission Complete \n");

  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
#endif
}

/*
 * Convert pulse durations to bits.
 * 
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 */
int convertTimingToBit(unsigned int t0, unsigned int t1)
{
  if (t0 > (BIT1_HIGH_MIN) && t0 < (BIT1_HIGH_MAX) && t1 > (BIT1_LOW_MIN) && t1 < (BIT1_LOW_MAX))
  {
    return 1;
  }
  else if (t0 > (BIT0_HIGH_MIN) && t0 < (BIT0_HIGH_MAX) && t1 > (BIT0_LOW_MIN) && t1 < (BIT0_LOW_MAX))
  {
    return 0;
  }
  return -1; // undefined
}

void handleNotFound(AsyncWebServerRequest *request)
{
  Serial.println("Received Not Found");
  Serial.print("uri:");
  Serial.println(request->url());
  request->send(404);
}

// void printRingbuff()
// {
//   char *response;
//   int itemID = 0;
//  unsigned int iCnt = resBuff->numElements(resBuff);

//   Serial.printf("Ringbuff contains %i elements \n", iCnt);

//   Serial.println("\n______Dumping contents of ring buffer_______");

//   while (resBuff->pull(resBuff, &response))
//   {
//     Serial.printf("Value of %i '", itemID);
//     Serial.printf("%s'\n",response);

//     itemID++;
//   }

//   Serial.println("\nEnd of Buffer");
// }

/*
 * helper code to print formatted hex 
 */
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
 char tmp[length*2+1];
 byte first;
 int j = 0;
 for (uint8_t i = 0; i < length; i++) 
 {
   first = (data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first ;
   j++;

   first = (data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (byte)39; 
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = 0;
 Serial.print(tmp);
}

/*
 *  Acurite Functions to process details
 *
 */
/*
 * Validate the CRC value to validate the packet
 *
 *
 */
bool acurite_crc(volatile byte row[])
{
  // sum of first n-1 bytes modulo 256 should equal nth byte
  int sum = 0;

  for (int i = 0; i < bytesReceived - 1; i++)
  {
    sum += row[i];
  }
  if (sum != 0 && sum % 256 == row[bytesReceived - 1])
  {
    return true;
  }
  else
  {

    Serial.print("Expected: ");
    Serial.print(row[bytesReceived - 1], HEX);
    Serial.print(" Got: ");
    Serial.println(sum % 256, HEX);
    return false;
  }
}

/*
 * Acurite 06045 Lightning sensor Temperature encoding
 * 12 bits of temperature after removing parity and status bits.
 * Message native format appears to be in 1/10 of a degree Fahrenheit
 * Device Specification: -40 to 158 F  / -40 to 70 C
 * Available range given encoding with 12 bits -150.0 F to +259.6 F
 */
float acurite_6045_getTemp(uint8_t highbyte, uint8_t lowbyte)
{
  int rawtemp = ((highbyte & 0x1F) << 7) | (lowbyte & 0x7F);
  float temp = (rawtemp - 1500) / 10.0;
  return temp;
}

float acurite_getTemp_6044M(byte hibyte, byte lobyte)
{
  // range -40 to 158 F
  int highbits = (hibyte & 0x0F) << 7;
  int lowbits = lobyte & 0x7F;
  int rawtemp = highbits | lowbits;
  float temp = (rawtemp / 10.0) - 100;
  return temp;
}

float convCF(float c)
{
  return ((c * 1.8) + 32);
}

int acurite_6045_strikeCnt(byte strikeByte)
{
  //int strikeTot = 0;
  //int strikeWrapOffset = 0;
  int strikeCnt = strikeByte & 0x7f;
  if (strikeTot == 0)
  {
    //Initialize Strike Counter
    strikeTot = strikeCnt;
    strikeWrapOffset = 0;
    strikeLast = millis();
    activeStrikes = false;
    return 0;
  }
  else if (strikeCnt < strikeTot && strikeCnt > 0)
  {
    /*Strikes wrapped around  
     *  Setting strikeTot to 1 as zero would cause a reset, 
     *   Need to make sure strikeCnt isn't 0 so we don't get 
     *   127 false strikes added to our wrap around
     */
    strikeWrapOffset = (127 - strikeTot) + strikeWrapOffset;
    strikeTot = 1;
    strikeLast = millis();
    activeStrikes = true;
  }
  else if (strikeCnt == strikeTot)
  {
    if (millis() - strikeLast > eventTimeoutms)
    {
      //Reset the Lightning event time its been more than the eventTiemoutms
      strikeTot = strikeCnt;
      strikeWrapOffset = 0;
      activeStrikes = false;
      //strikeLast = millis();
    }
  }
  else
  {
    //strike occured increase lastStrike
    strikeLast = millis();
    activeStrikes = true;
  }
  return (strikeCnt - strikeTot) + strikeWrapOffset;
}

uint8_t acurite_6045_strikeRange(uint8_t strikeRange)
{
  return strikeRange & 0x1f;
}

uint16_t acurite_txr_getSensorId(uint8_t hibyte, uint8_t lobyte)
{
  return ((hibyte & 0x3f) << 8) | lobyte;
}

int acurite_5n1_getBatteryLevel(uint8_t byte)
{
  return (byte & 0x40) >> 6;
}

int acurite_getHumidity(uint8_t byte)
{
  // range: 1 to 99 %RH
  int humidity = byte & 0x7F;
  return humidity;
}

float acurite_getWindSpeed_kph(uint8_t highbyte, uint8_t lowbyte)
{
  // range: 0 to 159 kph
  // raw number is cup rotations per 4 seconds
  // http://www.wxforum.net/index.php?topic=27244.0 (found from weewx driver)
  int highbits = (highbyte & 0x1F) << 3;
  int lowbits = (lowbyte & 0x70) >> 4;
  int rawspeed = highbits | lowbits;
  float speed_kph = 0;
  if (rawspeed > 0)
  {
    speed_kph = rawspeed * 0.8278 + 1.0;
  }
  return speed_kph;
}

char *getWindDirection_Descr(byte b)
{
  // 16 compass points, ccw from (NNW) to 15 (N),
  // { "NW", "WSW", "WNW", "W", "NNW", "SW", "N", "SSW",
  //   "ENE", "SE", "E", "ESE", "NE", "SSE", "NNE", "S" };
  int direction = b & 0x0F;
  return acurite_5n1_winddirection_str[direction];
}

float acurite_getTemp_5n1(byte highbyte, byte lowbyte)
{
  // range -40 to 158 F
  int highbits = (highbyte & 0x0F) << 7;
  int lowbits = lowbyte & 0x7F;
  int rawtemp = highbits | lowbits;
  float temp_F = (rawtemp - 400) / 10.0;
  return temp_F;
}

float acurite_getRainfall(uint8_t hibyte, uint8_t lobyte)
{
  // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
  int raincounter = ((hibyte & 0x7f) << 7) | (lobyte & 0x7F);
  if (acurite_5n1_raincounter > 0)
  {
    if (raincounter < acurite_5n1_raincounter)
    {
      rainWrapOffset = lastRainCount - acurite_5n1_raincounter;
      acurite_5n1_raincounter = 1;
      rainLast = millis();
      activeRain = true;
    }
    else if (acurite_5n1_raincounter == raincounter)
    {
      if ((millis() - rainLast) >= eventTimeoutms)
      {
        lastRainCount = raincounter;
        rainWrapOffset = 0;
        activeRain = false;
      }
    }
    else
    {
      rainLast = millis();
      activeRain = false;
    }
    return (raincounter - acurite_5n1_raincounter + rainWrapOffset) * .01;
  }
  else
  {
    acurite_5n1_raincounter = raincounter;
    lastRainCount = raincounter;
    rainLast = millis();
    activeRain = false;
    return 0.0;
  }
}

String getTimeSpan(unsigned long startMillis, unsigned long endMillis)
{
  String outString;
  long span = (endMillis - startMillis) / 1000;
  outString.concat(numberOfHours(span));
  outString.concat(":");
  outString.concat(numberOfMinutes(span));
  outString.concat(":");
  outString.concat(numberOfSeconds(span));
  return outString;
}

float convKphMph(float kph)
{
  return kph * 0.62137;
}

void decode_Acurite_6044(byte dataBytes[])
{
  char returnData[80];
  char tempBuff[5];

  sprintf(returnData, "id=%03x&ct=%08x&st=2", acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), millis());
  if ((dataBytes[4] & 0x20) == 0x20)
  {
    sprintf(returnData, "%s&bl=1", returnData);
  }
  dtostrf(convCF(acurite_getTemp_6044M(dataBytes[4], dataBytes[5])), 5, 1, tempBuff);
  sprintf(returnData, "%s&te=%s", returnData, tempBuff);
  sprintf(returnData, "%s&hu=%i", returnData, acurite_getHumidity(dataBytes[3]));

  resBuff->add(resBuff, &returnData);
}

void decode_5n1(byte dataBytes[])
{
  char returnData[80];
  char tempBuff[5];

  sprintf(returnData, "id=%03x&ct=%08x", acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), millis());
  if ((dataBytes[4] & 0x20) == 0x20)
  {
    sprintf(returnData, "%s&bl=1", returnData);
  }
  dtostrf(acurite_getWindSpeed_kph(dataBytes[3], dataBytes[4]), 5, 1, tempBuff);
  sprintf(returnData, "%s&ws=%s", returnData, tempBuff);

  if ((dataBytes[2] & 0x3F) == MT_WS_WD_RF)
  {
    sprintf(returnData, "%s&st=4", returnData);
    sprintf(returnData, "%s&wd=%s", returnData, getWindDirection_Descr(dataBytes[4]));
    dtostrf(acurite_getRainfall(dataBytes[5], dataBytes[6]), 5, 1, tempBuff);
    sprintf(returnData, "%s&ra=%s", returnData, tempBuff);
    sprintf(returnData, "%s&rt=%08x", returnData, rainLast);
  }
  else
  {
    sprintf(returnData, "%s&st=3", returnData);
    dtostrf(acurite_getTemp_5n1(dataBytes[4], dataBytes[5]), 5, 1, tempBuff);
    sprintf(returnData, "%s&te=%s", returnData, tempBuff);
    sprintf(returnData, "%s&hu=%i", returnData, acurite_getHumidity(dataBytes[6]));
  }

  resBuff->add(resBuff, &returnData);
}

void decode_Acurite_6045(byte dataBytes[])
{
  char returnData[80];
  char tempBuff[5];

  sprintf(returnData, "id=%03x&ct=%08x", acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), millis());
  sprintf(returnData, "%s&st=1", returnData);
  if ((dataBytes[4] & 0x20) == 0x20)
  {
    sprintf(returnData, "%s&bl=1", returnData);
  }
  dtostrf(acurite_6045_getTemp(dataBytes[4], dataBytes[5]), 5, 1, tempBuff);
  sprintf(returnData, "%s&te=", returnData, tempBuff);
  sprintf(returnData, "%s&hu=%i", returnData, acurite_getHumidity(dataBytes[3]));
  if ((dataBytes[7] & 0x20) == 0x20)
  {
    sprintf(returnData, "%s&li=1", returnData);
  }
  sprintf(returnData, "%s&ld=%i", returnData, acurite_6045_strikeRange(dataBytes[7]));
  sprintf(returnData, "%s&lc=%i", returnData, acurite_6045_strikeCnt(dataBytes[6]));
  sprintf(returnData, "%s&lt=%08x", returnData, strikeLast);

  resBuff->add(resBuff, &returnData);
}

// Print the bit stream for debugging.
// Generates a lot of chatter, normally disable this.
void displayBitTiming()
{
  unsigned int ringIndex;

  Serial.print("syncFound = ");
  Serial.println(syncFound);
  Serial.print("changeCount = ");
  Serial.println(changeCount);
  Serial.print("bytesReceived = ");
  Serial.println(bytesReceived);
  Serial.print("syncIndex = ");
  Serial.println(syncIndex);

  Serial.print("dataIndex = ");
  Serial.println(dataIndex);

  ringIndex = (syncIndex - (SYNCPULSEEDGES - 1)) % RING_BUFFER_SIZE;

  for (int i = 0; i < (SYNCPULSECNT + (bytesReceived * 8)); i++)
  {
    int bit = convertTimingToBit(pulseDurations[ringIndex % RING_BUFFER_SIZE],
                                 pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);

    Serial.print("bit ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(bit);
    Serial.print(" t1 = ");
    Serial.print(pulseDurations[ringIndex % RING_BUFFER_SIZE]);
    Serial.print(" t2 = ");
    Serial.println(pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);

    ringIndex += 2;
  }
}

void loop(void)
{
  char tempBuff[85];

  if (received == true)
  {
    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(1);

    // extract temperature value
    unsigned int startIndex, stopIndex, ringIndex;
    unsigned long temperature = 0;
    bool fail = false;

//define DISPLAY_BIT_TIMING
#ifdef DISPLAY_BIT_TIMING
    displayBitTiming();
#endif // DISPLAY_BIT_TIMING

    //Decode to Hex Bytes
    byte dataBytes[bytesReceived];
    fail = false; // reset bit decode error flag

    // clear the data bytes array
    for (int i = 0; i < bytesReceived; i++)
    {
      dataBytes[i] = 0;
    }

    ringIndex = (syncIndex + 1) % RING_BUFFER_SIZE;

    for (int i = 0; i < bytesReceived * 8; i++)
    {
      int bit = convertTimingToBit(pulseDurations[ringIndex % RING_BUFFER_SIZE],
                                   pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);

      if (bit < 0)
      {
        fail = true;
        break; // exit loop
      }
      else
      {
        dataBytes[i / 8] |= bit << (7 - (i % 8));
      }

      ringIndex += 2;
    }

// Display the raw data received in hex
#define DISPLAY_DATA_BYTES
#ifdef DISPLAY_DATA_BYTES

    if (fail)
    {
      Serial.println("Data Byte Display : Decoding error.");
    }
    else
    {
      for (int i = 0; i < bytesReceived; i++)
      {
        PrintHex8(&dataBytes[i], 1);
        Serial.print(",");
      }
    }

#endif

    if (fail)
    {
      //Do Nothing if failed
    }
    else
    {

      if (resBuff->isFull(resBuff))
      {
        //Serial.println("Buff Full Popping top");
        //popping the top to get rid of it.
        resBuff->pull(resBuff, &tempBuff);
      }

      if (bytesReceived == 7)
      {
        //Small Tower sensor with Temp and Humidity Only
        decode_Acurite_6044(dataBytes);
      }
      else if (bytesReceived == 8)
      {
        //5n1 tower sensor
        decode_5n1(dataBytes);
      }
      else if (bytesReceived == 9)
      {
        //Lightening detector
        decode_Acurite_6045(dataBytes);
      }
    }
    // delay for 1 second to avoid repetitions
    delay(1000);
    received = false;
    syncFound = false;
    // re-enable interrupt
    attachInterrupt(1, handler, CHANGE);
  } // received==true

  // if (micros() % 36 == 0)
  // {
  //   //Serial.println("Preparing to add entry");
  //   if (resBuff->isFull(resBuff))
  //   {
  //     //Serial.println("Buff Full Popping top");
  //     //popping the top to get rid of it.
  //     resBuff->pull(resBuff, &tempBuff);
  //   }
  //   //Serial.println("Creating Data");
  //   tempBuff = createData();
  //   //Serial.println("Adding Data");
  //   resBuff->add(resBuff, &tempBuff);
  //   //Serial.println("Add Complete");
  // }

  // if (millis() % 45 == 0)
  // {
  //   if (resBuff->isEmpty(resBuff))
  //   {
  //     Serial.println("Ring Buffer is currently empty");
  //   }
  //   else
  //   {
  //     printRingbuff();
  //   }
  // }
}