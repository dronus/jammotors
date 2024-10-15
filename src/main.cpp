
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <ArtnetnodeWifi.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <Preferences.h>


#define statusLedPin 2
#define RX_PIN 4
#define TX_PIN 5
uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;

Preferences prefs;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;

bool dmx_enabled = true;
int homing = 0;
int enabled=0;
int32_t target = 0;

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
  {Param::INT, "speed", [](Param& p) { cybergear.set_limit_speed(prefs.getInt(p.name,10000)/1000.f);}},
  {Param::INT, "accel", [](Param& p) { cybergear.set_limit_current(enabled * prefs.getInt("accel",10000)/1000.f); }},
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
  Serial.print(WiFi.softAPIP());
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

uint32_t manual_target=0;
uint32_t artnet_target=0;

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }
  // if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
  //   Serial.println("Alert: The Transmission was successful.");
  //   Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
  // }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}



void setup() 
{
  pinMode(statusLedPin,OUTPUT);

  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(prefs.getInt("speed",10000)/1000.f); /* set the maximum speed of the motor */
  cybergear.set_limit_current(prefs.getInt("poweron_enable",0) * prefs.getInt("accel",10000)/1000.f);
  cybergear.set_speed_kp(0.1f);
  cybergear.enable_motor(); /* turn on the motor */
  cybergear.set_position_ref(0.0); /* set initial rotor position */
  Serial.println("Cybergear started.");


//  Serial.begin(115200);
  Serial.println("NF Motor - WiFi ArtNet stepper motor driver with web interface");

  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  // reset WiFi credentials
  // prefs.putString("ssid","nf-9G"), prefs.putString("psk","brunxxer");

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
    //if (request->hasParam("set_position") )
    //  stepper->setCurrentPosition(request->getParam("set_position")->value().toInt());
    if (request->hasParam("enable") )
      enabled = request->getParam("enable")->value().toInt();
    if (request->hasParam("home") )
      homing = request->getParam("home")->value().toInt();
    if (request->hasParam("reset") )
      ESP.restart();

    cybergear.set_limit_current(enabled * prefs.getInt("accel",10000)/1000.f); 
  });

  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("%s %d\n","target",target);
    response->printf("%s %d\n","position",(int32_t)round(cybergear.get_status().position*1000.f));
    response->printf("%s %d\n","enable",enabled);
    response->printf("%s %d\n","alarm",0);
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
  target = manual_target;

  // add artnet commanded target
  target += artnet_target;
 
  // add oscillatory movement 
  osc_phase += prefs.getInt("osc_f") * dt * 2.f * 3.14159f / 1000.f / 1000.f;
  osc_phase = fmod(osc_phase, ((float)PI * 2.f));
  target += floor( prefs.getInt("osc_a") * sin(osc_phase) );
  
  // add random movement
  random_countdown -= dt;
  if(random_countdown < 0) {
    random_target    = random(prefs.getInt("random_a"));
    random_countdown = prefs.getInt("random_d") + random(prefs.getInt("random_rd"));
  }
  target += random_target;

  // execute move
  
  cybergear.set_position_ref(target / 1000.f);


  // cybergear.request_status();
  // check_alerts();
  // XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  // Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  delay(100);
}
