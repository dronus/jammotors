
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <ArtnetnodeWifi.h>
#include "wifi_config.h" // provides wifiName, wifiSecret

#include "FastAccelStepper.h"

#define statusLedPin 2
#define stepPin 4
#define dirPin 16

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;

void setup() 
{
  pinMode(statusLedPin,OUTPUT);
  Serial.begin(115200);

  bool spiffsBeginSuccess = SPIFFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("SPIFFS cannot be mounted.");
  else
    Serial.println("SPIFFS started.");

  Serial.println("Connecting WIFi");
  WiFi.begin(wifiName, wifiSecret);
  //ESPServerWifiModeClient
  int cycle = 0;
  while ( WiFi.status() != WL_CONNECTED )
  {
      Serial.print(".");
      delay(500);
      if ( cycle % 10 == 0) {
        WiFi.reconnect();
        Serial.println();
      }
      cycle++;
  }
  Serial.println();
  Serial.print("connected, IP address: ");
  Serial.println(WiFi.localIP().toString().c_str());

  httpServer = new AsyncWebServer(80);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

  httpServer->on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("speed"))
      stepper->setSpeedInHz(request->getParam("speed")->value().toInt());
    if (request->hasParam("accel"))
      stepper->setAcceleration(request->getParam("accel")->value().toInt());
    if (request->hasParam("target") )
      stepper->moveTo(request->getParam("target")->value().toInt());
    request->send(204);
  });

  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("%d %d %d %d\n", 
      stepper->targetPos(),
      stepper->getCurrentPosition(),
      stepper->getSpeedInMilliHz() / 1000,
      stepper->getAcceleration()
    );
    
    request->send(response);
  });
  httpServer->serveStatic("/", SPIFFS, "/"); // serve index.html and other files
  httpServer->begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    uint16_t target = ((uint16_t)data[0]) + (((uint16_t)data[1])<<8);
    Serial.println(target, DEC);
    stepper->moveTo(target);
  });
  artnetnode.begin();
    Serial.println("ArtNet node started.");
  
  // start stepper driver
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin,1);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    Serial.println("Stepper started.");
  }

  // light status led
  digitalWrite(statusLedPin,HIGH);
}

void loop() 
{
  artnetnode.read();
}
