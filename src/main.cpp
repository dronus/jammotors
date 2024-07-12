
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <ArtnetnodeWifi.h>
#include "wifi_config.h" // provides wifiName, wifiSecret
#include "FastAccelStepper.h"
#include <Preferences.h>


#define statusLedPin 2
#define stepPin 17
#define dirPin  16
#define enablePin 21
#define alarmPin 22

Preferences prefs;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
int dmxChannel, scale;
int homing = 0;

void setup() 
{
  pinMode(statusLedPin,OUTPUT);
  pinMode(enablePin,OUTPUT);
  digitalWrite(enablePin,HIGH);
  pinMode(alarmPin,INPUT_PULLUP);
  Serial.begin(115200);

  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  dmxChannel = prefs.getInt("channel",0);
  scale = prefs.getInt("scale",1);
  Serial.println("Prefs started.");

  Serial.println("Connecting WIFi");
  WiFi.setHostname("Motor");
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
    if (request->hasParam("speed")) {
      stepper->setSpeedInHz(request->getParam("speed")->value().toInt());
      prefs.putInt("speed", request->getParam("speed")->value().toInt());
    }
    if (request->hasParam("accel")) {
      stepper->setAcceleration(request->getParam("accel")->value().toInt());
      prefs.putInt("accel",    request->getParam("accel")->value().toInt());
    }
    if (request->hasParam("channel")) {
      dmxChannel = request->getParam("channel")->value().toInt();
      prefs.putInt("channel",request->getParam("channel")->value().toInt());
    }
    if (request->hasParam("scale")) {
      scale = request->getParam("scale")->value().toInt();
      prefs.putInt("scale", request->getParam("scale")->value().toInt());
    }

    if (request->hasParam("target") )
      stepper->moveTo(request->getParam("target")->value().toInt());

    if (request->hasParam("set_position") )
      stepper->setCurrentPosition(request->getParam("set_position")->value().toInt());

   if (request->hasParam("enable") )
      digitalWrite(enablePin, request->getParam("enable")->value().toInt() );

   if (request->hasParam("home") )
     homing = request->getParam("home")->value().toInt();

   request->send(204);

  });

  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("%d %d %d %d %d %d %d %d\n",
      dmxChannel,
      scale,
      stepper->targetPos(),
      stepper->getSpeedInMilliHz() / 1000,
      stepper->getAcceleration(),
      stepper->getCurrentPosition(),
      digitalRead(enablePin),
      !digitalRead(alarmPin)
    );
    
    request->send(response);
  });
  httpServer->serveStatic("/", LittleFS, "/"); // serve index.html and other files
  httpServer->begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    if(dmxChannel + 1 >= length) return;
    uint32_t target = scale * ( ((uint32_t)data[0+dmxChannel]) + (((uint32_t)data[1+dmxChannel])<<8) );
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
    stepper->setSpeedInHz   (prefs.getInt("speed",10000));
    stepper->setAcceleration(prefs.getInt("accel",10000));

    Serial.println("Stepper started.");
  }

  // light status led
  digitalWrite(statusLedPin,HIGH);
}

void loop() 
{
  artnetnode.read();

  if(homing) {
    // do poor man's "bump" homing
    // we move slowly towards the specified position, until the driver errors out.
    // we expect to have hit a hard stop then, and reset the current position to zero.
    dmxChannel = 255; // illegal channel, to disable dmx temporary
    stepper->setSpeedInHz(prefs.getInt("speed",10000)/ 10); // slow and careful
    stepper->moveTo(homing);
    while(digitalRead(alarmPin)) // wait for alarm to come up
      delay(10); 
    digitalWrite(enablePin,0);   // clear the alarm
    stepper->forceStop(); // stop homing movement
    delay(500);
    stepper->setCurrentPosition(0);
    digitalWrite(enablePin,1);  // alarm is clear, ready to go again.
    stepper->setSpeedInHz(prefs.getInt("speed",10000)); // restore speed
    homing = 0;
    dmxChannel = prefs.getInt("channel",0);
  }
}
