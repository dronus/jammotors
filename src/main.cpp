
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <ArtnetnodeWifi.h>
#include "wifi_config.h" // provides wifiName, wifiSecret

#include "FastAccelStepper.h"


//#define dirPinStepper 18
//#define enablePinStepper 26
//#define stepPinStepper 17

#define dirPinStepper 2
#define enablePinStepper 6
#define stepPinStepper 4


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
ArtnetnodeWifi artnetnode;

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
    int speed = 50000;
    int accel = 10000;
    int target = 0;
    if (request->hasParam("speed"))
      speed = request->getParam("speed")->value().toInt();
    if (request->hasParam("accel"))
      accel = request->getParam("accel")->value().toInt();
    if (request->hasParam("target") )
      target = request->getParam("target")->value().toInt();
    
    // TODO do move
    Serial.print("MOVE: ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(accel);
    Serial.println(" ");

    stepper->setSpeedInUs(1000000L / speed);  // us/step
    stepper->setAcceleration(accel);
    stepper->moveTo(target);
    
    request->send(204);
  });

  httpServer->on("/position", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("%d\n", stepper->getCurrentPosition());
    request->send(response);
  });

  httpServer->serveStatic("/", SPIFFS, "/");

  httpServer->begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    uint16_t target = ((uint16_t)data[0]) + (((uint16_t)data[1])<<8);
    Serial.println(target, DEC);
    stepper->moveTo(target);
  });
  artnetnode.begin();

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper,1);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    // stepper->setEnablePin(enablePinStepper);
    // stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

  }
  Serial.println("Stepper started.");
}

void loop() 
{
  artnetnode.read();
}
