/*
  ESP32 WiFi universal motor controller.
  
  Can control different motors by DMX / ArtNet, HTTP and a simple Web UI featuring some motion generators.
  
  Supported drives:
  -Stepper motors with step / dir interface
  -RC servo motors with PPM interface
  -DC motors and lights with PWM interface
  -Xiaomi Cybergear geared servo motors with CAN bus interface

  Motors can be controlled either remotely in realtime with DMX or (soon) OSC commands or the Web UI,
  or by internal motion generators (oscillations, random movements) configured by the Web UI.
  
  Config can be stored permanently to run simple movements directly after power up.

  If no wifi as configured is found, an access point is automatically created. 
  To access the Web UI, just connect with the "Motor" acces point, and use a browser to access
  
    http://192.168.4.1/
  
  After a valid wifi access configuration is set and connected to, the device can only be found by looking 
  at your routers client table or using nmap or other tools.
  
  The remote control and motion generator in the Web UI can be tested with no drives attached
  by setting a channel to "PWM" driver on pin 2 - this will control the ESP32 module's onboard LED.

*/
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
#include "axis.h"
#include "driver_stepper.h"
#include "driver_servo.h"
#include "driver_pwm.h"
#include "driver_cybergear.h"
#include "midi_picker.h"

Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;


#include <WiFiUdp.h>
WiFiUDP udpIn;
unsigned int oscInPort = 8888;
MicroOscUdp<1024> oscReceiver(&udpIn, IPAddress(0,0,0,0), 0);
MidiPicker midi_picker;

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
  P_int32_t (ik_length_c,true,    0,  1000,  90);
  P_int32_t (ik_length_a,true,    0,  1000, 330);
  P_int32_t (ik_length_b,true,    0,  1000, 390);
  P_int32_t (ik_vel_max,true,    0,  10000,  500);
  P_int32_t (ik_vel_k  ,true,    0, 100000, 5000);
  P_int32_t (ik_acc_max,true,    0, 100000, 5000);
  P_end;
} global_params;

struct Status : public Params {
  P_int32_t (dt, false, 0, 0, 0);
  P_int32_t (dt_max, false, 0, 0, 0);
  P_int32_t (ik_error, false, 0, 0, 0);
  P_end;
} status;

Axis axes[4];

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

  for(Param* p = params->getParams(); p ; p = p->next()) {

    char prefs_name[32];
    if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
    else
      snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);

    if (request->hasParam(prefs_name) ) {
      int32_t value = request->getParam(prefs_name)->value().toInt();
      p->set(value);
      if(p->desc->persist)
        prefs.putInt(prefs_name, value);
    }
  }
}


float ik_x_dmx_target = 0, ik_y_dmx_target = 0 , ik_z_dmx_target = 0;

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

  // initialize LittleFS on flash for persistent parameter storage by Prefs library
  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  readPrefs(&global_params);
  readPrefs(&midi_picker);

  for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
    Channel& channel = channels[channel_id];
    // read preferences
    readPrefs(&channel,channel_id);
    // initialize
    channel.init();
  }
  for(uint8_t axis_id = 0; axis_id<3; axis_id++)
    readPrefs(&axes[axis_id],axis_id);

  // try to connect configured WiFi AP. If not possible, back up 
  // and provide own AP, to allow further configuration.
  Serial.println("Connecting WIFi");
  WiFi.setHostname(prefs.getString("name","Motor").c_str());

  if(prefs.getString("psk","")=="")
    WiFi.begin(prefs.getString("ssid"));
  else
    WiFi.begin(prefs.getString("ssid"), prefs.getString("psk"));
  
  // wait for connection to establish
  int cycle = 1;
  while ( WiFi.status() != WL_CONNECTED )
  {
      Serial.print(".");
      delay(500);
      if (cycle >= 10) {
        // no succes after 5 seconds, back up and provide own access point.
        Serial.println();
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
    readFromRequest(&midi_picker, -1, request);

    // check for axes parameters
    for(uint8_t i=0; i<3; i++)
      readFromRequest(&axes[i], i, request);

    if (request->hasParam("set_can_id") ) {
      uint8_t new_can_id = request->getParam("set_can_id")->value().toInt();
      static_cast<DriverCybergear*>(channels[channel_id].driver)->writeId(new_can_id);
    }

    // handle instantaneous commands
    if (request->hasParam("home") )
      homing = request->getParam("home")->value().toInt();
    if (request->hasParam("reset") )
      ESP.restart();
  });

  // http "status" API to query current status
  httpServer->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    int channel_id = request->hasParam("channel_id") ? request->getParam("channel_id")->value().toInt() : 0;    
    for(Param* p = channels[channel_id].getParams(); p; p=p->next())
      if(!p->desc->persist) response->printf("%s_%d %d\n",p->desc->name,channel_id,(int32_t)p->get());
    for(Param* p = global_params.getParams(); p; p=p->next())    
      if(!p->desc->persist) response->printf("%s %d\n",p->desc->name,(int32_t)p->get());
    for(Param* p = status.getParams(); p; p = p->next())
      response->printf("%s %d\n",p->desc->name, (int32_t)(p->get()));
    for(uint8_t i=0; i<3; i++)
      for(Param* p = axes[i].getParams(); p; p = p->next())
        if(!p->desc->persist) response->printf("%s_%d %d\n",p->desc->name, i, (int32_t)(p->get()));

    request->send(response);
  });

  // http "config" API to query persistent configuration
  httpServer->on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    
    // send global configuration
    for(char* param : global_string_params)
      response->printf("%s %s\n",param, prefs.getString(param,"").c_str());
    for(Param* p = global_params.getParams(); p; p = p->next())
      if(p->desc->persist) response->printf("%s %d\n",p->desc->name, (int32_t)(p->get()));
    for(Param* p = midi_picker.getParams(); p; p = p->next())
      if(p->desc->persist) response->printf("%s %d\n",p->desc->name, (int32_t)(p->get()));

    // send current channel configuration
    int channel_id = request->hasParam("channel_id") ? request->getParam("channel_id")->value().toInt() : 0;
    for(Param* p = channels[channel_id].getParams(); p; p = p->next())
      if(p->desc->persist) response->printf("%s_%d %d\n",p->desc->name, channel_id, (int32_t)(p->get()));

    // send axes configuration
    for(uint8_t i=0; i<3; i++)
      for(Param* p = axes[i].getParams(); p; p = p->next())
        if(p->desc->persist) response->printf("%s_%d %d\n",p->desc->name, i, (int32_t)(p->get()));

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
    
    for(uint8_t i=0; i<3; i++)
      if(axes[i].ik_dmx_ch && axes[i].ik_dmx_ch <= length)
        axes[i].ik_dmx_target = axes[i].ik_dmx_a / 255.f * (float)data[axes[i].ik_dmx_ch-1];

  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");

  // start OSC Udp receiver
  udpIn.begin(oscInPort);
  Serial.println("OSC receiver started.");
 
  // light status led
  digitalWrite(statusLedPin,HIGH);
}

float fm_osc(float o, float a, float f, float fb, uint32_t dt, float &phase) {

  phase += f * dt * 2.f * (float)PI / 1000.f / 1000.f;
  phase = fmod(phase, ((float)PI * 2.f));

  return o + a * sin( phase + fb / 1000.f * sin(phase) );
}

float update_ik_axis(Axis& axis, uint32_t dt) {
  axis.target += fm_osc(axis.ik_offset + axis.ik_dmx_target, axis.ik_osc_a, axis.ik_osc_f, axis.ik_osc_fb, dt, axis.ik_phase);

  float vel = ( axis.target - axis.pos ) * global_params.ik_vel_k / 1000.f;
  vel = min( vel,  global_params.ik_vel_max * 1.f);
  vel = max( vel, -global_params.ik_vel_max * 1.f);

  float acc = ( vel - axis.vel ) / ( dt / 1000.f);
  float acc_limit =  global_params.ik_acc_max;
  // if ( acc * vel < 0) acc_limit *= 5.f;   // allow for stronger braking to prevent overshoot
  acc = min( acc,  acc_limit);
  acc = max( acc, -acc_limit);

  axis.vel += acc * dt / 1000.f;
  axis.pos += axis.vel * dt / 1000.f;

  return axis.pos;
}

void update_ik(uint32_t dt) {

  if(channels[0].ik_a == 0 && channels[1].ik_a == 0 && channels[2].ik_a == 0) 
    // for testing, enable IK even if only a single channel has IK mixed in.
    return;

  float x = update_ik_axis(axes[0],dt);
  float y = update_ik_axis(axes[1],dt);
  float z = update_ik_axis(axes[2],dt);

  // define shoulder - target angle
  if(y == 0 && x == 0) return; // if y and z are zero, just keep last angles.
 
  // define xy-plane origin - shoulder - target triangle
  float rot = sqrtf(x*x+y*y); // span origin to target in xy-plane
  float ros = global_params.ik_length_c; // shoulder offset
  if(rot < ros) return; // target to close
  float rst = sqrtf(rot*rot - ros*ros); // offset shoulder to target distance
  float alpha = atan2(x,y) - acos ((rot*rot + rst*rst - ros*ros) / (2 * rot * rst));
  channels[0].ik_target = alpha  / (float)pi;

  // get elbow angle by triangle cosine equation 
  float a = global_params.ik_length_a; // upper arm
  float b = global_params.ik_length_b; // lower arm
  float c = sqrtf(rst*rst + z*z); // span to target
  if(c>a+b) return; // point out of reach - arm to short.
  if(b>a+c) return; // point out of reach - lower arm to long.
  if(a>b+c) return; // point out of reach - upper arm to long.
  float gamma = acos ((a*a + b*b - c*c) / (2 * a * b)) - pi;
  
  // get shoulder angle by triangle cosine equation and atan offset
  if(z==0 && rst==0) return; // point undefined.
  float beta  = acos ((a*a + c*c - b*b) / (2 * a * c)) - atan2(rst,z);
  channels[1].ik_target = beta  / (float)pi;
  channels[2].ik_target = gamma / (float)pi;
}


void rot(float _x_in, float _y_in, float alpha, float& x_out, float& y_out) {
  float x_in = _x_in;
  float y_in = _y_in;
  x_out =  x_in * cos(alpha) - y_in * sin(alpha);
  y_out =  x_in * sin(alpha) + y_in * cos(alpha);
}

void update_ik_feedback() {
  
  float alpha = channels[0].position / (float)channels[0].ik_a * (float)pi;
  float beta  = channels[1].position / (float)channels[1].ik_a * (float)pi;
  float gamma = channels[2].position / (float)channels[2].ik_a * (float)pi;

  float x=0, y=0, z=0;
  z += global_params.ik_length_b;
  rot(y,z,gamma,y,z);
  z += global_params.ik_length_a;
  rot(y,z,beta,y,z);
  x += global_params.ik_length_c;
  rot(y,x,alpha,y,x);

  axes[0].ik_feedback = x;
  axes[1].ik_feedback = y;
  axes[2].ik_feedback = z;

  float dx = axes[0].pos - x;
  float dy = axes[1].pos - y;
  float dz = axes[2].pos - z;

  status.ik_error = sqrtf( dx*dx + dy*dy + dz*dz );
}

void oscMessageParser( MicroOscMessage& receivedOscMessage) {
  Serial.printf("OSC in : %s",receivedOscMessage.buffer);

  if ( receivedOscMessage.checkOscAddress("/midi") ) {
    const uint8_t* midi;
    receivedOscMessage.nextAsMidi(&midi);    
    uint8_t type = midi[0] & 0xF0;
    Serial.printf("OSC MIDI cmd/channel:%d note:%d velocity:%d ",midi[0],midi[1],midi[2]);
    if(type == 0x90) {// note on
      Serial.printf("note_on ",midi[0],midi[1],midi[2]);
      midi_picker.pick(midi[1]);
    }
    Serial.println();
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
  status.dt = time - last_time;
  if(last_time != 0) 
    status.dt_max = max(status.dt, status.dt_max);
  last_time = time;

  for(Axis& axis : axes)
    axis.target = 0;
  midi_picker.update(axes,status.dt);
  update_ik(status.dt);
  update_ik_feedback(); // to get IK error

  for(Channel& c : channels)
    c.update(status.dt);

  delay(10);
}
