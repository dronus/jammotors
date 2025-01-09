
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h>
#include <ArtnetnodeWifi.h>
#include <Preferences.h>

const uint8_t max_channels = 4;
const uint32_t statusLedPin = 2;
const float pi = 3.1415926f;

#include "param.h"
#include "driver.h"
#include "channel.h"
#include "driver_stepper.h"
#include "driver_servo.h"
#include "driver_pwm.h"
#include "driver_cybergear.h"


Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;


#include <WiFiUdp.h>
WiFiUDP udpIn;
unsigned int oscInPort = 8888;
MicroOscUdp<1024> oscReceiver(&udpIn, IPAddress(0,0,0,0), 0);


int homing = 0;

Channel channels[max_channels];

Driver* createDriver(uint8_t driver_id, uint8_t pin_id) {
  Serial.printf("Create driver %d on pin / CAN id %d\n",driver_id,pin_id);
  if(driver_id == 1) return new DriverStepper();
  if(driver_id == 2) return new DriverServo(pin_id);
  if(driver_id == 3) return new DriverPWM(pin_id);
  if(driver_id == 4) return new DriverCybergear(pin_id);

  return NULL;
}

char* global_string_params[] = {
  "name",
  "ssid",
  "psk"
};

struct GlobalParams : public Params {
  P_int32_t (ik_length_a,    0,  1000, 330);
  P_int32_t (ik_length_b,    0,  1000, 390);
  P_int32_t (ik_x,       -1000,  1000, 0);
  P_int32_t (ik_x_osc_a, -1000,  1000, 0);
  P_int32_t (ik_x_osc_f,     0, 10000, 1000);
  P_int32_t (ik_x_osc_fb,    0,  2000, 0);
  P_int32_t (ik_y,       -1000,  1000, 0);
  P_int32_t (ik_y_osc_a, -1000,  1000, 0);
  P_int32_t (ik_y_osc_f,     0, 10000, 1000);
  P_int32_t (ik_y_osc_fb,    0,  2000, 0);
  P_int32_t (ik_z,       -1000,  1000, 720);
  P_int32_t (ik_z_osc_a, -1000,  1000, 0);
  P_int32_t (ik_z_osc_f,     0, 10000, 1000);
  P_int32_t (ik_z_osc_fb,    0,  2000, 0);
  P_end;
} global_params;

// switch WiFi to acces point mode and provide an captive portal page.
// this allows configuration of WiFi credentials in case the 
// configured one is unavailable.
void switchWifiAp() {
  Serial.println("Switching WiFi to AP mode.");
  WiFi.disconnect();
  WiFi.softAP(prefs.getString("name","Motor").c_str());
  Serial.printf("AP created: %s, %s \n",prefs.getString("name","Motor").c_str(),WiFi.softAPIP().toString().c_str());
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

void readPrefs(Params* params, int16_t channel_id = -1) {
  for(Param* p = params->getParams(); p; p=p->next()) {
    char prefs_name[32];
    if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
    else
      snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);

    Serial.print(prefs_name);Serial.print(" ");
    if(prefs.isKey(prefs_name)) {
      int val = prefs.getInt(prefs_name); 
      Serial.print(val);
      p->set(val);
    } else {
      Serial.print(" use default ");
      Serial.print(p->get());
    }
    Serial.println();
  }
}

void readFromRequest(Params* params, int16_t channel_id, AsyncWebServerRequest *request) {
  for(Param* p = params->getParams(); p ; p = p->next())
    if (request->hasParam(p->desc->name) ) {
      int32_t value = request->getParam(p->desc->name)->value().toInt();
      p->set(value);
      char prefs_name[32];
      if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
        snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
      else
        snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);
      prefs.putInt(prefs_name, value);
    }
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("NF Motor - WiFi ArtNet stepper motor driver with web interface");

  // status LED, start off and turn on once initalisation is complete.
  pinMode(statusLedPin,OUTPUT);

  // make sure WiFi won't autoconnect and store connections on itself
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.persistent(false);

  // initialize LittleFS on flash for persitent parameter storage by Prefs library
  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  readPrefs(&global_params);

  for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
    Channel& channel = channels[channel_id];
    
    // read preferences
    readPrefs(&channel,channel_id);

    // initialize
    channel.init();
  }

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

    readFromRequest(&channels[channel_id], channel_id, request);
    
    // check for global parameters
    readFromRequest(&global_params, -1, request);
    // check for global string parameters
    for(char* param : global_string_params)
      if (request->hasParam(param) )
        prefs.putString(param, request->getParam(param)->value());

    if (request->hasParam("set_can_id") ) {
      uint8_t new_can_id = request->getParam("set_can_id")->value().toInt();
      static_cast<DriverCybergear*>(channels[channel_id].driver)->writeId(new_can_id);
    }

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
    response->printf("%s %d\n","target",channels[channel_id].target);
    response->printf("%s %d\n","position",channels[channel_id].position);
    response->printf("%s %d\n","torque",channels[channel_id].torque);
    response->printf("%s %d\n","temperature",channels[channel_id].temperature);
    response->printf("%s %d\n","enable",channels[channel_id].enabled);
    response->printf("%s %d\n","alarm",channels[channel_id].have_alarm);
    request->send(response);
  });

  // http "config" API to query persistent configuration
  httpServer->on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    
    // send global configuration
    for(char* param : global_string_params)
      response->printf("%s %s\n",param, prefs.getString(param,"").c_str());

    for(Param* p = global_params.getParams(); p; p = p->next())
      response->printf("%s %d\n",p->desc->name, (int32_t)(p->get()));

    // send current channel configuration
    int channel_id = request->hasParam("channel_id") ? request->getParam("channel_id")->value().toInt() : 0;    
    for(Param* p = channels[channel_id].getParams(); p; p = p->next())
      response->printf("%s %d\n",p->desc->name, (int32_t)(p->get()));
      
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
      if(c.dmx_channel > 0 && c.dmx_channel-1 < length)
        c.artnet_target = c.scale * ((uint32_t)data[c.dmx_channel-1]);
    }

  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");

  // start OSC Udp receiver
  udpIn.begin(oscInPort);
  
 
  // light status led
  digitalWrite(statusLedPin,HIGH);
}

float fm_osc(float o, float a, float f, float fb, uint32_t dt, float &phase) {

  phase += f * dt * 2.f * (float)PI / 1000.f / 1000.f;
  phase = fmod(phase, ((float)PI * 2.f));

  return o + a * sin( phase + fb / 1000.f * sin(phase) );
}

float ik_phase_x=0, ik_phase_y=0, ik_phase_z=0;

void update_ik(uint32_t dt) {

  if(channels[0].ik_a == 0 && channels[1].ik_a == 0 && channels[2].ik_a == 0) 
    // for testing, enable IK even if only a single channel has IK mixed in.
    return;

  float x = fm_osc(global_params.ik_x, global_params.ik_x_osc_a, global_params.ik_x_osc_f, global_params.ik_x_osc_fb, dt, ik_phase_x);
  float y = fm_osc(global_params.ik_y, global_params.ik_y_osc_a, global_params.ik_y_osc_f, global_params.ik_y_osc_fb, dt, ik_phase_y);
  float z = fm_osc(global_params.ik_z, global_params.ik_z_osc_a, global_params.ik_z_osc_f, global_params.ik_z_osc_fb, dt, ik_phase_z);
  
  // define shoulder - target angle
  if(y != 0 || x != 0) {
    float alpha = atan2(x,y);
    channels[0].ik_target = alpha  / (float)pi;
  } // if y and z are zero, just keep last rotation angle.

  // define shoulder - elbow - target triangle
  float a = global_params.ik_length_a; // upper arm
  float b = global_params.ik_length_b; // lower arm
  float c = sqrtf(x*x+y*y+z*z); // span to target

  // get elbow angle by triangle cosine equation 
  if(c>a+b) return; // point out of reach - arm to short.
  float gamma = acos ((a*a + b*b - c*c) / (2 * a * b)) - pi;
  
  // get shoulder angle by triangle cosine equation and atan offset
  float d_xy = sqrtf(x*x+y*y); // distance in xy-plane to z-axis
  if(b>a+c) return; // point out of reach - lower arm to long.
  if(z==0 && d_xy==0) return; // point 0,0,0 undefined.
  float beta  = acos ((a*a + c*c - b*b) / (2 * a * c)) - atan2(d_xy,z);

  channels[1].ik_target = beta  / (float)pi;
  channels[2].ik_target = gamma / (float)pi;
}


void oscMessageParser( MicroOscMessage& receivedOscMessage) {
  Serial.printf("OSC in : %s",receivedOscMessage.buffer);

  if ( receivedOscMessage.checkOscAddress("/note") ) {
    Serial.println("OSC Note in");
    const uint8_t* midi;
    receivedOscMessage.nextAsMidi(&midi);
    Serial.printf("%d %d %d %d",midi[0],midi[1],midi[2],midi[3]);
  }
}

uint32_t last_time;

void loop() 
{
  ArduinoOTA.handle();
  if(WiFi.getMode() == WIFI_MODE_AP)
    dnsServer.processNextRequest();
  artnetnode.read();
  oscReceiver.onOscMessageReceived( oscMessageParser );

  // compute delta time
  uint32_t time = millis();
  uint32_t dt = time - last_time;
  last_time = time;
  //Serial.println(dt);

  update_ik(dt);

  for(Channel& c : channels)
    c.update(dt);
  
  delay(10);
}
