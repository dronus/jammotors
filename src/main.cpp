
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

// pins and ids for Cybergear CAN bus ("TWAI" in ESP32)
#define RX_PIN 4
#define TX_PIN 5
uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;

Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;

bool dmx_enabled = true;
bool have_alarm=0;
int homing = 0;
int enabled=0;
int32_t target = 0;

// type for a settable, storable parameter.
// May hold an "apply" method that should be called after changing the parameter.
// Provides "trySet" to check a webserver request for changes in this parameter.
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

struct Channel {
  XiaomiCyberGearDriver cybergear;
  Channel(int _can_id) : cybergear(_can_id,MASTER_CAN_ID) {}
  void init_motor() {
    // initialize CyberGear on CAN bus
    cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
    cybergear.init_motor(MODE_POSITION);
    cybergear.set_position_kp(prefs.getInt("position_kp",1000)/1000.f);
    cybergear.enable_motor(); /* turn on the motor */
    cybergear.set_position_ref(0.0); /* set initial rotor position */
    Serial.println("Cybergear started.");
  }
  void update() {
    cybergear.set_limit_speed(prefs.getInt("speed",10000)/1000.f);
    cybergear.set_position_kp(prefs.getInt("position_kp",1000)/1000.f);
    cybergear.set_limit_current(enabled * prefs.getInt("accel",10000)/1000.f);
    // TODO cybergear.set_position_ref(0.0); 
  }
};

Channel channels[] = {
  Channel(CYBERGEAR_CAN_ID)
};

// list of settable and stored parameters
Param params[] = {
  {Param::INT, "poweron_enable" },
  {Param::INT, "speed"},
  {Param::INT, "position_kp"},
  {Param::INT, "accel"},
  {Param::INT, "channel" },
  {Param::INT, "scale" },
  {Param::INT, "osc_f" },
  {Param::INT, "osc_a" },
  {Param::INT, "random_d" },
  {Param::INT, "random_rd" },
  {Param::INT, "random_a" }
  ,
//};

//Param global_params[] = {
  {Param::STRING, "name" },
  {Param::STRING, "ssid" },
  {Param::STRING, "psk" }
};

// switch WiFi to acces point mode and provide an captive portal page.
// this allows configuration of WiFi credentials in case the 
// configured one is unavailable.
// TODO dosn't work stable for now - often no DHCP on this one fails 
// and no IP address is obtained. 
// Maybe because of broad bandwith which collides with every other WiFi?
void switchWifiAp() {
  Serial.println("Switching WiFi to AP mode.");
  WiFi.softAP(prefs.getString("name","Motor").c_str(), "motorkraft3000");
  Serial.print(WiFi.softAPIP());
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// intermediate position targets set by manual input or incoming artnet messages
uint32_t manual_target=0;
uint32_t artnet_target=0;

// receive incoming CAN messages from CyberGear motor and forward them to driver
static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    channels[0].cybergear.process_message(message);
  }
}

// check alerts from CAN bus, receive incoming messages from CyberGear motor
static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    have_alarm = true;
    enabled = false;
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
  // status LED, start off and turn on once initalisation is complete.
  pinMode(statusLedPin,OUTPUT);

  enabled = prefs.getInt("poweron_enable",0);

  for(Channel& channel : channels) 
    channel.init_motor();

//  Serial.begin(115200);
  Serial.println("NF Motor - WiFi ArtNet stepper motor driver with web interface");

  // initialize LittleFS on flash for persitent parameter storage by Prefs library
  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  // reset WiFi credentials
  // prefs.putString("ssid","nf-9G"), prefs.putString("psk","brunxxer");

  // try to connect configured WiFi AP. If not possible, back up 
  // and provide own AP, to allow further configuration.
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

  // initialize OTA receiver for receiving firmware updates over WiFi
  Serial.println("Start OTA receiver.");
  ArduinoOTA.onStart([]()                  { Serial.println("OTA Start updating"   );});
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Error[%u]: ", error);});
  ArduinoOTA.begin();

  // http Webserver for receiving commands
  httpServer = new AsyncWebServer(80);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

  // http "set" API to set and apply values
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
    if (request->hasParam("enable") ) {
      enabled = request->getParam("enable")->value().toInt();
      if(have_alarm) {
        channels[0].init_motor();
        have_alarm = 0;
      }
    }
    if (request->hasParam("home") )
      homing = request->getParam("home")->value().toInt();
    if (request->hasParam("reset") )
      ESP.restart();

    channels[0].cybergear.set_limit_current(enabled * prefs.getInt("accel",10000)/1000.f); 
  });

  // http "status" API to query current status
  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    
     XiaomiCyberGearStatus status = channels[0].cybergear.get_status();
  // Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
    
    response->printf("%s %d\n","target",target);
    response->printf("%s %d\n","position",(int32_t)round(status.position*1000.f));
    response->printf("%s %d\n","torque",(int32_t)round(status.torque*1000.f));
    response->printf("%s %d\n","temperature",(int32_t)status.temperature);
    response->printf("%s %d\n","enable",enabled);
    response->printf("%s %d\n","alarm",have_alarm);
    request->send(response);
  });

  // http "config" API to query persistent configuration
  httpServer->on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    for(Param& p : params)
      if(p.type == Param::INT)
        response->printf("%s %d\n",p.name,prefs.getInt(p.name));
      else if(p.type == Param::STRING)
        response->printf("%s %s\n",p.name, prefs.getString(p.name).c_str());
    request->send(response);
  });

  // serve index.html single page UI
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
uint32_t last_status;
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
  //Serial.println(dt);

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
  channels[0].update();
  channels[0].cybergear.set_position_ref(target/1000.f);
 
  // check motor state
  // also updates motor status data that can be relayed to wifi clients then.
  check_alerts();

  delay(10);
}
