
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <ArtnetnodeWifi.h>
#include "FastAccelStepper.h"
#include <Preferences.h>



#define statusLedPin 2
#define stepPin 16
#define dirPin  17
#define enablePin 21
#define alarmPin 22

Preferences prefs;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;

bool dmx_enabled = true;
int homing = 0;


struct Param {
  enum ParamType {INT, STRING} type;
  const char* name;
  void (*apply)(Param&) = NULL;

  Param(ParamType _type, const char* _name, void(*_apply)(Param&) = NULL) : type(_type), name(_name),apply(_apply) {};

  void trySet(AsyncWebServerRequest* request) {
    if(! request->hasParam(name) ) return;
    if(type == INT)
      prefs.putInt(name, request->getParam(name)->value().toInt());
    else if(type == STRING)
      prefs.putString(name, request->getParam(name)->value());
    if(apply != NULL)
      (*apply)(*this);
  }
};

Param params[] = {
  {Param::INT, "poweron_enable" },
  {Param::INT, "speed", [](Param& p) { stepper->setSpeedInHz(prefs.getInt(p.name)); }},
  {Param::INT, "accel", [](Param& p) { stepper->setAcceleration(prefs.getInt(p.name)); }},
  {Param::INT, "channel" },
  {Param::INT, "scale" },
  {Param::INT, "osc_f" },
  {Param::INT, "osc_a" },
  {Param::INT, "random_d" },
  {Param::INT, "random_rd" },
  {Param::INT, "random_a" },
  {Param::STRING, "name" },
  {Param::STRING, "ssid" },
  {Param::STRING, "psk" }
};


void switchWifiAp() {
  // provide a WiFi AP with an captive portal page.
  // this allows configuration of WiFi credentials in case the 
  // configured one is unavailable.
  Serial.println("Switching WiFi to AP mode.");
  WiFi.softAP(prefs.getString("name","Motor").c_str(), "motorkraft3000");
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

uint32_t manual_target=0;
uint32_t artnet_target=0;

// set external pin handler
// as default FastAccelStepper interrupt handling collides
// with flash read/write used by Prefs etc.
bool setExternalPin(uint8_t pin, uint8_t value) {
  pin = pin & ~PIN_EXTERNAL_FLAG;
  pinMode(pin, OUTPUT);
  bool oldValue = digitalRead(pin);
  digitalWrite(pin, value);
  return oldValue;
}

void setup() 
{
  pinMode(statusLedPin,OUTPUT);
  pinMode(enablePin,OUTPUT);
  pinMode(alarmPin,INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("NF Motor - WiFi ArtNet stepper motor driver with web interface");

  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  Serial.println("Connecting WIFi");
  WiFi.setHostname(prefs.getString("name","Motor").c_str());
  if(prefs.getString("psk","")=="")
    WiFi.begin(prefs.getString("ssid"));
  else
    WiFi.begin(prefs.getString("ssid"), prefs.getString("psk"));
  //ESPServerWifiModeClient
  int cycle = 1;
  while ( WiFi.status() != WL_CONNECTED )
  {
      Serial.print(".");
      delay(500);
      if ( cycle % 10 == 0) {
        //WiFi.reconnect();
        Serial.println();
      }
      if (cycle >= 10) {
        switchWifiAp();
        break;
      }
      cycle++;
  }
  Serial.println();
  Serial.print("connected, IP address: ");
  Serial.println(WiFi.localIP().toString().c_str());

  Serial.println("Start OTA receiver.");
  ArduinoOTA.onStart([]()                  { Serial.println("OTA Start updating"   );});
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Error[%u]: ", error);});
  ArduinoOTA.begin();

  httpServer = new AsyncWebServer(80);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

  httpServer->on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {

    request->send(204);

    // handle persistent preferences
    for(Param& p : params)
      p.trySet(request);
    
    // handle instantaneous commands
    if (request->hasParam("target") )
      manual_target = request->getParam("target")->value().toInt();
    if (request->hasParam("set_position") )
      stepper->setCurrentPosition(request->getParam("set_position")->value().toInt());
    if (request->hasParam("enable") )
      digitalWrite(enablePin, request->getParam("enable")->value().toInt() );
    if (request->hasParam("home") )
      homing = request->getParam("home")->value().toInt();
    if (request->hasParam("reset") )
      ESP.restart();
  });

  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("%s %d\n","target",stepper->targetPos());
    response->printf("%s %d\n","position",stepper->getCurrentPosition());
    response->printf("%s %d\n","enable",digitalRead(enablePin));
    response->printf("%s %d\n","alarm",!digitalRead(alarmPin));
    request->send(response);
  });

  httpServer->on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    for(Param& p : params)
      if(p.type == Param::INT)
        response->printf("%s %d\n",p.name,prefs.getInt(p.name));
      else if(p.type == Param::STRING)
        response->printf("%s %s\n",p.name, prefs.getString(p.name).c_str());
    request->send(response);
  });

  httpServer->serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  httpServer->begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    if(!dmx_enabled) return;
    uint8_t dmxChannel = prefs.getInt("channel");
    if(dmxChannel + 1 >= length) return;
    artnet_target = prefs.getInt("scale") * ( ((uint32_t)data[0+dmxChannel]) + (((uint32_t)data[1+dmxChannel])<<8) );
  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");

  // start stepper driver
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin,DRIVER_MCPWM_PCNT);
  if (stepper) {
    stepper->setDirectionPin( dirPin + PIN_EXTERNAL_FLAG ); // use external pin handler
    engine.setExternalCallForPin(setExternalPin);

    stepper->setSpeedInHz   (prefs.getInt("speed",10000));
    stepper->setAcceleration(prefs.getInt("accel",10000));

    Serial.println("Stepper started.");

    digitalWrite(enablePin,prefs.getInt("poweron_enable",0));
  }

  // light status led
  digitalWrite(statusLedPin,HIGH);
}

uint32_t last_time;
float osc_phase = 0;
int32_t random_countdown = 0;
int32_t random_target = 0;

void loop() 
{
  ArduinoOTA.handle();
  if(WiFi.getMode() == WIFI_MODE_AP)
    dnsServer.processNextRequest();
  artnetnode.read();

  // compute delta time
  uint32_t time = millis();
  uint32_t dt = time - last_time;
  last_time = time;

  // compute and set motion target
  // set manual target (offset)
  int32_t target = manual_target;

  // add artnet commanded target
  target += artnet_target;
 
  // add oscillatory movement 
  osc_phase += prefs.getInt("osc_f") * dt * 2.f * 3.14159f / 1000.f / 1000.f;
  target += floor( prefs.getInt("osc_a") * sin(osc_phase) );
  
  // add random movement
  random_countdown -= dt;
  if(random_countdown < 0) {
    random_target    = random(prefs.getInt("random_a"));
    random_countdown = prefs.getInt("random_d") + random(prefs.getInt("random_rd"));
  }
  target += random_target;

  // execute move
  stepper->moveTo(target);


  if(homing) {
    // do poor man's "bump" homing
    // we move slowly towards the specified position, until the driver errors out.
    // we expect to have hit a hard stop then, and reset the current position to zero.
    dmx_enabled = false;
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
    dmx_enabled = true;
  }
  
  delay(1);
}
