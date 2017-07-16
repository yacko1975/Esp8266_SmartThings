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

#define WLAN_SSID "CarterNet"
#define WLAN_PASS "cat-fish"
#define ws_Port 8151

//Enable/disable functions
#define Enable_Wifi 1

AsyncWebServer server(ws_Port);

IPAddress stHubIP(192, 168, 20, 28);
int stPort = 80;
HTTPClient hClient;

RingBuf *resBuff = RingBuf_new(80, 20);

void setup(void)
{

  Serial.begin(115200);
  Serial.println("Blind Startup Sequence");
  Serial.println();

#ifdef Enable_Wifi
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
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

  server.on("/test", HTTP_ANY, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "test:success");
    response->addHeader("Server", "ESP Async Web Server");
    request->send(response);

    int params = request->params();
    char param[85];

    for (int i = 0; i < params; i++)
    {
      AsyncWebParameter *p = request->getParam(i);
      if (!p->isFile() & !p->isPost())
      {
        if (i > 0)
        {
          sprintf(param, "%s&", param);
        }
        sprintf(param, "%s%s=%s", param, p->name().c_str(), p->value().c_str());
      }
    }

    Serial.print("Parameters are: ");
    Serial.println(param);

    delay(2000);

    //clientReqeust(param);
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
#endif
}

// #ifdef Enable_Wifi
// void clientReqeust(char *param)
// {
//   Serial.println("Connection to Hub");
//   if (hClient.begin(stHubIP.toString(), ws_Port, "/"))
//   {
//     hClient.addHeader("Host", WiFi.localIP().toString() + ws_Port);
//     hClient.addHeader("Content-Type", "text/plain");


//         int httpCode = hClient.POST(param);
//     Serial.println("http code" + (String)httpCode);
//     Serial.println("Response: " + hClient.getString());
//     hClient.end();
//   }
// }

// #endif

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

char *createData()
{

  switch (millis() % 12)
  {
  case 0:
  case 4:
  case 8:
    return (char *)"id=HHH&ct=HHHHHHHH&st=1&bl=1&te=999.9&hu=999&lc=99999&lr=99&lt=HHHHHHHH&li=1";
    break;
  case 1:
  case 5:
  case 9:
    return (char *)"id=HHH&ct=HHHHHHHH&st=1&bl=1&te=999.9&hu=100";
    break;
  case 2:
  case 6:
  case 10:
    return (char *)"id=HHH&ct=HHHHHHHH&st=1&bl=1&te=999.9&hu=100&ws=999.9";
    break;
  default:
    return (char *)"id=HHH&ct=HHHHHHHH&st=1&bl=1&wd=AAA&ra=999.9&rl=HHHHHHHH";
  }
}

void loop(void)
{
  char *tempBuff;

//Serial.printf("\nStart Heap: %i\n", ESP.getFreeHeap());

  //delay(random(100,2000));

  if (micros() % 36 == 0)
  {
    Serial.println("Preparing to add entry");
    if (resBuff->isFull(resBuff))
    {
      Serial.println("Buff Full Popping top");
      //popping the top to get rid of it.
      resBuff->pull(resBuff, &tempBuff);
    }
    Serial.println("Creating Data");
    tempBuff = createData();
    Serial.println("Adding Data");    
    resBuff->add(resBuff, &tempBuff);
    Serial.println("Add Complete");
  }

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
  //Serial.printf("\nEnd Heap: %i\n",  ESP.getFreeHeap());
}