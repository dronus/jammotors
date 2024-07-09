
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include "wifi_config.h" // provides wifiName, wifiSecret

AsyncWebServer * httpServer;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Checking SPIFFS");
  bool spiffsBeginSuccess = SPIFFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("SPIFFS cannot be mounted.");
  else
    Serial.println("SPIFFS started.");

  Serial.println("Connecting WIFi");
  WiFi.begin(wifiName, wifiSecret);
  //ESPServerWifiModeClient
  int timeoutCounter = 10;
  while (WiFi.status() != WL_CONNECTED && timeoutCounter > 0)
  {
      delay(500);
      Serial.print(".");
      if (timeoutCounter == (5 * 2 - 3))
      {
          WiFi.reconnect();
      }
      timeoutCounter--;
  }
  Serial.println();
  if (timeoutCounter > 0)
  {
    Serial.print("Connected to network with IP address ");
    Serial.println(WiFi.localIP().toString().c_str());
  }
  else
  {
    Serial.print("Connection to WiFi network "); 
    Serial.print(wifiName);
    Serial.println("failed with timeout");
  }

  httpServer = new AsyncWebServer(80);

  // this->restApiHandler->registerRestEndpoints(this->httpServer);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

/*  httpServer->onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS)
      request->send(200);
    else
      request->send(404, "text/html", "Requested file not found!");
  });
*/
  httpServer->on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
    float speed = 100.f;
    float accel = 100.f;
    float decel = 100.f;
    float distance = 100.f;
    if (request->hasParam("speed"))
      speed = request->getParam("speed")->value().toFloat();
    if (request->hasParam("accel"))
      accel = request->getParam("accel")->value().toFloat();
    if (request->hasParam("decel"))
      decel = request->getParam("decel")->value().toFloat();
    if (request->hasParam("distance") )
      distance = request->getParam("distance")->value().toFloat();
    
    // TODO do move
    Serial.print("MOVE: ");
    Serial.print(distance);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(accel);
    Serial.print(" ");
    Serial.print(decel);
    Serial.println(" ");
    request->send(204);
  });

  httpServer->serveStatic("/", SPIFFS, "/");

  httpServer->begin();
  Serial.println("Webserver started.");
}

void loop() 
{
}
