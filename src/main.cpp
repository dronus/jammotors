
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <ArtnetnodeWifi.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <Preferences.h>
#include "param.h"

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

int homing = 0;

struct Channel : public Params {
  XiaomiCyberGearDriver cybergear;
  Channel(int _can_id) : cybergear(_can_id,MASTER_CAN_ID), can_id(_can_id){}

  P_int32_t (poweron_enable, 0, 1, 0);
  P_int32_t (speed, 0, 100000, 10000);
  P_int32_t (position_kp, 0, 10000, 1000);
  P_int32_t (accel, 0, 100000, 10000);
  P_int32_t (channel, 0, 255, 0);
  P_int32_t (scale, 0, 10000,  0);
  P_int32_t (osc_f, 0, 10000,  0);
  P_int32_t (osc_a, 0, 100000, 0);
  P_int32_t (random_d, 0, 100000, 1000);
  P_int32_t (random_rd,0, 100000, 1000);
  P_int32_t (random_a ,0, 10000, 0);

  int can_id;
  int enabled;
  int have_alarm;
  int32_t target = 0;
  int32_t manual_target=0;
  int32_t artnet_target=0;
  float osc_phase = 0;
  int32_t random_countdown = 0;
  int32_t random_target = 0;

  void init() {
    // initialize CyberGear on CAN bus    
    cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
    cybergear.init_motor(MODE_POSITION);
    cybergear.set_position_kp(position_kp/1000.f);
    cybergear.enable_motor(); /* turn on the motor */
    cybergear.set_position_ref(0.0); /* set initial rotor position */
    enabled = poweron_enable;
    Serial.println("Cybergear started.");
  }
  void update(uint32_t dt) {
    cybergear.set_limit_speed(speed/1000.f);
    cybergear.set_position_kp(position_kp/1000.f);
    cybergear.set_limit_current(enabled * accel/1000.f);

    // compute and set motion target
    // set manual target (offset)
    target = manual_target;

    // add artnet commanded target
    target += artnet_target;
   
    // add oscillatory movement 
    osc_phase += osc_f * dt * 2.f * 3.14159f / 1000.f / 1000.f;
    osc_phase = fmod(osc_phase, ((float)PI * 2.f));
    target += floor( osc_a * sin(osc_phase) );
    
    // add random movement
    random_countdown -= dt;
    if(random_countdown < 0) {
      random_target    = random(random_a);
      random_countdown = random_d + random(random_rd);
    }
    target += random_target;

    // execute move
    cybergear.set_position_ref(target/1000.f);
  }
};

const uint8_t max_channels = 1;
Channel channels[max_channels] = {
  Channel(CYBERGEAR_CAN_ID)
};

char* global_params[] = {
  "name",
  "ssid",
  "psk"
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

// receive incoming CAN messages from CyberGear motor and forward them to driver
static void handle_rx_message(twai_message_t& message) {
  for(Channel& channel : channels) {
    if (((message.identifier & 0xFF00) >> 8) == channel.can_id){
      channel.cybergear.process_message(message);
    }
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
    channels[0].have_alarm = true;
    channels[0].enabled = false;
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

  for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
    Channel& channel = channels[channel_id];
    
    // read preferences
    Param* p = channel.getParams();
    do {
      char prefs_name[32];
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);      
      p->set(prefs.getInt(prefs_name, p->desc->def));
    } while ( p = p->next() );
    // initialize
    channel.init();
  }

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

    int channel_id = request->hasParam("channel_id") ? request->getParam("channel_id")->value().toInt() : 0;

    // handle persistent per-channel parameters
    
    for(Param* p = channels[channel_id].getParams(); p ; p = p->next())
      if (request->hasParam(p->desc->name) ) {
        p->set(request->getParam(p->desc->name)->value().toInt());
        char prefs_name[32];
        snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
        prefs.putInt(prefs_name, request->getParam(p->desc->name)->value().toInt());
      }
      
    // check for global parameters
    for(char* param : global_params)
      if (request->hasParam(param) )
        prefs.putString(param, request->getParam(param)->value());

    // handle instantaneous commands
    if (request->hasParam("target") )
      channels[channel_id].manual_target = request->getParam("target")->value().toInt();
    //if (request->hasParam("set_position") )
    //  stepper->setCurrentPosition(request->getParam("set_position")->value().toInt());
    if (request->hasParam("enable") ) {
      channels[channel_id].enabled = request->getParam("enable")->value().toInt();
      if(channels[channel_id].have_alarm) {
        channels[channel_id].init();
        channels[channel_id].have_alarm = 0;
      }
    }
    if (request->hasParam("home") )
      homing = request->getParam("home")->value().toInt();
    if (request->hasParam("reset") )
      ESP.restart();
  });

  // http "status" API to query current status
  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");

    int channel_id = request->hasParam("channel_id") ? request->getParam("channel_id")->value().toInt() : 0;    
    XiaomiCyberGearStatus status = channels[channel_id].cybergear.get_status();
    response->printf("%s %d\n","target",channels[channel_id].target);
    response->printf("%s %d\n","position",(int32_t)round(status.position*1000.f));
    response->printf("%s %d\n","torque",(int32_t)round(status.torque*1000.f));
    response->printf("%s %d\n","temperature",(int32_t)status.temperature);
    response->printf("%s %d\n","enable",channels[channel_id].enabled);
    response->printf("%s %d\n","alarm",channels[channel_id].have_alarm);
    request->send(response);
  });

  // http "config" API to query persistent configuration
  httpServer->on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    for(char* param : global_params)
      response->printf("%s %s\n",param, prefs.getString(param,"").c_str());
    request->send(response);
  });

  // serve index.html single page UI
  httpServer->serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  httpServer->begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
      Channel& c = channels[channel_id];
      if(c.channel < length)
        c.artnet_target = c.scale * ( ((uint32_t)data[0+c.channel]) + (((uint32_t)data[1+c.channel])<<8) );
    }

  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");
 
  // light status led
  digitalWrite(statusLedPin,HIGH);
}

uint32_t last_time;

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

  for(Channel& c : channels)
    c.update(dt);
 
  // check motor state
  // also updates motor status data that can be relayed to wifi clients then.
  check_alerts();

  delay(10);
}
