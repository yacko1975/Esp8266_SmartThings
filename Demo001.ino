

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


#define WLAN_SSID       "XXXX"
#define WLAN_PASS       "XXXX"
#define ws_Port         8151

AsyncWebServer server(ws_Port);

IPAddress stHubIP(192, 168, 20, 28);
 int stPort = 80;
HTTPClient hClient;


void setup(void) {

  Serial.begin(115200);
  Serial.println("Blind Startup Sequence");
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(500);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Listening on Port:");
  Serial.println(ws_Port);

  server.on("/refresh", HTTP_ANY,[](AsyncWebServerRequest *request){
    Serial.println("Received Refresh");
    Serial.print("Arguments Received:");
    int params = request->params();
    
    Serial.println(params);
    
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }
    
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "refresh:success");
    response->addHeader("Server","ESP Async Web Server");
    request->send(response);
  });

  server.on("/test", HTTP_ANY,[](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "test:success");
    response->addHeader("Server","ESP Async Web Server");
    request->send(response);

    int params = request->params();
    char param[80];

    for(int i = 0; i<params; i++)
    {
      AsyncWebParameter* p = request->getParam(i);
      if(!p->isFile() & !p->isPost()){
         if (i > 0){
          sprintf(param, "%s&", param);
        } 
        sprintf(param, "%s%s=%s",param,  p->name().c_str(),  p->value().c_str());
      }
    }

    
    Serial.print("Parameters are: " );
    Serial.println( param);

    delay(2000);
    
    clientReqeust(param);
  });
  
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}

void clientReqeust(char *param){
  Serial.println("Connection to Hub");
  if (hClient.begin(stHubIP.toString(),ws_Port, "/"))
  {
   hClient.addHeader("Host", WiFi.localIP().toString() + ws_Port);
   hClient.addHeader("Content-Type", "text/plain");
   hClient.addHeader("")

   int httpCode = hClient.POST(param);
     Serial.println("http code" + (String)httpCode);
     Serial.println("Response: " + hClient.getString());
    hClient.end();
  }
  
}

void handleNotFound(AsyncWebServerRequest *request){
  Serial.println("Received Not Found");
  Serial.print("uri:");
  Serial.println(request->url());
  request->send(404);

  
}



void loop(void) {
 
}