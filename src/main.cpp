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
#include "esp_wifi.h"
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h>
#include <ArtnetnodeWifi.h>
#include <Preferences.h>

const uint32_t statusLedPin = 2;
const float pi = 3.1415926f;

#include "param.h"
#include "driver.h"
#include "channel.h"
#include "axis.h"
#include "motion.h"
#include "kinematic.h"
#include "driver_stepper.h"
#include "driver_servo.h"
#include "driver_pwm.h"
#include "driver_cybergear.h"
#include "midi_picker.h"
#include "cue.h"


Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer httpServer(80);
AsyncWebSocket ws("/ws");

DNSServer dnsServer;


#include <WiFiUdp.h>
WiFiUDP udp;
unsigned int oscInPort = 8888;
MicroOscUdp<1024> oscUdp(&udp);
MidiPicker midi_picker;
MotionController controller;

int homing = 0;

const uint8_t max_channels = 5;
std::vector<Channel> channels(max_channels);
const uint8_t max_axes = max_channels + 3;
std::vector<Axis> axes(max_axes);
const uint8_t max_cues = 4;
std::vector<Cue> cues; 

Driver* createDriver(uint8_t driver_id, uint8_t pin_id) {
  Serial.printf("Create driver %d on pin / CAN id %d\n",driver_id,pin_id);
  if(driver_id == 1) return new DriverStepper();
  if(driver_id == 2) return new DriverServo(pin_id);
  if(driver_id == 3) return new DriverPWM(pin_id);
  if(driver_id == 4) return new DriverCybergear(pin_id);

  return NULL;
}

Kinematic* createKinematic(uint8_t kinematic_id) {
  Serial.printf("Create kinematic %d \n",kinematic_id);
  if(kinematic_id == 1) return new KinematicArmCartesian();
  if(kinematic_id == 2) return new KinematicHand();
  
  return NULL;
}

struct Status : public Params {
  P_float (dt, false, 0, 0, 0);
  P_float (dt_max, false, 0, 0, 0);
  P_float (ik_error, false, 0, 0, 0);
  P_uint8_t (send_status,false,0,1,0);
  P_float(vbus,false,0,1,0);
  P_float(voltage_divider, true, 0, 64000, 10000);
  P_uint32_t (uptime, false, 0, 0, 0);
  P_string (name, true, "Motor");
  P_string (ssid, true, " "); // crashes with empty default string "" - why ?
  P_string (psk,  true, " "); // crashes with empty default string "" - why ?
  P_string   (osc_out_ip  ,  true, "0.0.0.0"); // crashes with empty default string "" - why ?
  P_uint32_t (osc_out_port,  true, 0, 0xFFFF, 8888);
  P_uint32_t (nvs_free, false, 0, 0,0 );
  P_uint32_t (fs_free, false, 0, 0,0 );
  P_uint32_t (ram_free, false, 0, 0,0 );
  P_bool(reset, false, false);
  P_end;
} status;
  
// switch WiFi to acces point mode and provide an captive portal page.
// this allows configuration of WiFi credentials in case the 
// configured one is unavailable.
void switchWifiAp() {
  Serial.println("Switching WiFi to AP mode.");
  WiFi.disconnect();
  WiFi.softAP(status.name.c_str());
  Serial.printf("AP created: %s, %s \n",status.name.c_str(),WiFi.softAPIP().toString().c_str());
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

bool readPrefs(Params* params, int16_t channel_id = -1) {
  bool got_some = false;
  for(Param* p = params->getParams(); p; p=p->next()) {
    
    if(!p->desc->persist) continue;
    
    char prefs_name[32];
    if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
    else
      snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);

    Serial.print(prefs_name);Serial.print(" ");

    if (prefs.isKey(prefs_name)){      
      if(p->desc->type == P_STRING) {
        String val = prefs.getString(prefs_name); // readPrefsString(prefs_name,p->getString().c_str());
        Serial.print(val);
        p->set(std::string(val.c_str()));
      } else { 
        float val = prefs.getFloat(prefs_name);
        Serial.print(val);
        p->set(val);
      }
      got_some = true;
    }

    Serial.println();
  }
  
  return got_some;
}

// set the parameter matching "key" from the given Params struct to "value".
void setParam(Params* params, int16_t channel_id, char* key, char* value_str, bool dont_save = false) {

  for(Param* p = params->getParams(); p ; p = p->next()) {

    char prefs_name[32];
    if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
    else
      snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);

    if ( strcmp(prefs_name, key) == 0 ) {
      // Serial.printf("setParam %s : %s\n", prefs_name, value_str);
      p->set(value_str);
      if(!dont_save && p->desc->persist)
        if(p->desc->type == P_STRING)
          prefs.putString(prefs_name,value_str);
        else
          prefs.putFloat(prefs_name, p->get());
    }
  }
}

// set single parameter from incoming key, value message.
void setFromWs(char* key_value, bool dont_save = false)  {
  // Serial.printf("Set %s\n",key_value);
  char* key       = key_value;
  char* value_str = strchr(key_value,' ');
  if(!value_str) return; // no value
  *(value_str++) = 0; // mark end of key and advance
  // Serial.printf("Set %s %s\n",key,value_str);

  // handle persistent per-channel parameters
  for(uint8_t channel_id = 0; channel_id < channels.size(); channel_id++)
    setParam(&channels[channel_id], channel_id, key, value_str, dont_save);
  
  // check for global parameters
  setParam(&status, -1,  key, value_str, dont_save);
  setParam(&controller, -1,  key, value_str, dont_save);
  if(controller.kinematic) setParam(controller.kinematic, -1,  key, value_str, dont_save);
  setParam(&midi_picker, -1,  key, value_str, dont_save);

  // check for axes parameters
  for(uint8_t i=0; i<axes.size(); i++)
    setParam(&axes[i], i,  key, value_str, dont_save);

  // check for cue parameters
  for(uint8_t i=0; i<cues.size(); i++)
    setParam(&cues[i], i,  key, value_str, dont_save);
};

void writeParamsToBuffer(char*& ptr, Params& params, bool persistent, int8_t index=-1) {
  for(Param* p = params.getParams(); p; p = p->next())
    if(p->desc->persist == persistent) {
      ptr += (size_t)sprintf(ptr,"%s", p->desc->name);
      if(index != -1) // add index to name, if set
        ptr += (size_t)sprintf(ptr,"_%d", index);
      if(p->desc->type != P_STRING)  // if number
        ptr += (size_t)sprintf(ptr," %.5g\n",p->get());
      else {// if string
        ptr += (size_t)snprintf(ptr,256," %s\n",p->getString().c_str());
      }
    }
}

template <typename Type> void writeParamsArrayToBuffer (char*& ptr, std::vector<Type>& params, bool persistent) {
  for(int8_t i=0; i<params.size(); i++)
    writeParamsToBuffer(ptr, params[i], persistent, i );
}

size_t writeAllParamsToBuffer(char* buffer, bool persistent) {
  char* ptr = buffer;
  writeParamsArrayToBuffer(ptr, channels, persistent);
  writeParamsToBuffer(ptr, controller, persistent);
  if(controller.kinematic) writeParamsToBuffer(ptr, *(controller.kinematic), persistent);
  writeParamsToBuffer(ptr, status, persistent);
  writeParamsToBuffer(ptr, midi_picker, persistent);
  writeParamsArrayToBuffer(ptr, axes, persistent);
  writeParamsArrayToBuffer(ptr, cues, persistent);
  return ptr - buffer;
}

// WebSocket "status" API to send current status
void send_status() {
  size_t len = 2048;
  char buffer[len];
  size_t written = writeAllParamsToBuffer(buffer, false);
  if(written >= len) Serial.printf("DEBUG send_status BUFFER OVERFLOW! bytes: %d \n", written ); 
  ws.textAll(buffer);
}


void send_osc() {
  if(!status.osc_out_port)
    return;
  // send current position of all axes as OSC MIDI message, if configured
  for(Axis& axis : axes)
    if(axis.ik_midi_cc) {
      float pos = (axis.ik_feedback - axis.ik_offset) / (float)axis.ik_midi_a * 128.f;
      pos = max(0.f,min(127.f,pos));
      int32_t midi_data = (0xB0<<24) + (axis.ik_midi_cc<<16) + (((uint8_t)pos)<<8); // - 2**32
      oscUdp.sendInt("/midi",midi_data);
    }
    
  // send current IK position as native OSC message  
  uint8_t axe_idxs[] = {0,1,2,6}; // x,y,z,a
  float osc_pos[4];
  for(uint8_t i=0; i<4; i++)
    osc_pos[i] = axes[axe_idxs[i]].ik_feedback - axes[axe_idxs[i]].ik_offset;
  if(status.osc_out_port)
    oscUdp.sendMessage("/xyza","ffff",osc_pos[0],osc_pos[1],osc_pos[2],osc_pos[3]);
}

uint32_t last_time;
const uint32_t cycle_time = 10;
void motionLoop(void* dummy){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  last_time = millis();
  while(true) {
    vTaskDelayUntil( &xLastWakeTime, cycle_time );
    // compute delta time
    uint32_t time = millis();
    status.uptime = time / 1000;
    uint32_t dt = time - last_time;
    last_time = time;
    status.dt = dt / 1000.f;
    if(status.uptime > 5)
      status.dt_max = max(status.dt, status.dt_max);

    for(Axis& axis : axes)
      axis.ik_input = 0;
    midi_picker.update(axes,status.dt);
    status.ik_error = controller.update(status.dt, axes, channels);
    
    for(Channel& c : channels)
      c.update(status.dt);
    
    send_osc();
  }
}

void setup() 
{
  // status LED, start off and turn on once initalisation is complete.
  pinMode(statusLedPin,OUTPUT);
  digitalWrite(statusLedPin,LOW);

  Serial.begin(115200);
  Serial.println("NF Motor - WiFi ArtNet stepper motor driver with web interface");

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
  
  readPrefs(&status);
  readPrefs(&controller);
  if(controller.kinematic) readPrefs(controller.kinematic);
  readPrefs(&midi_picker);

  for(uint8_t channel_id = 0; channel_id<channels.size(); channel_id++) {
    Channel& channel = channels[channel_id];
    // read preferences
    readPrefs(&channel,channel_id);
    // initialize
    channel.init();
  }
  for(uint8_t axis_id = 0; axis_id<axes.size(); axis_id++)
    readPrefs(&axes[axis_id],axis_id);

  for(uint8_t cue_id = 0, got_one = true; cue_id<max_cues && got_one; cue_id++) {
    cues.push_back(Cue());
    got_one = readPrefs(&cues[cue_id],cue_id);
  }

  // try to connect configured WiFi AP. If not possible, back up 
  // and provide own AP, to allow further configuration.
  Serial.println("Connecting WIFi");
  
  if(status.psk=="")
    WiFi.begin(status.ssid.c_str());
  else
    WiFi.begin(status.ssid.c_str(), status.psk.c_str());
  
  // wait for connection to establish
  int cycle = 1;
  while ( WiFi.status() != WL_CONNECTED )
  {
      Serial.print(".");
      delay( 500 + random(500) );
      if (cycle >= 10) {
        // no succes after 5 seconds, back up and provide own access point.
        Serial.println();
        switchWifiAp();
        break;
      }
      cycle++;
  }
  WiFi.setHostname(status.name.c_str());
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.println();
  Serial.print("connected, IP address: ");
  Serial.println(WiFi.localIP().toString().c_str());

  // initialize OTA receiver for receiving firmware updates over WiFi
  Serial.println("Start OTA receiver.");
  ArduinoOTA.onStart([]()                  { Serial.println("OTA Start updating"   );});
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Error[%u]: ", error);});
  ArduinoOTA.begin();

  // set mDNS announcement - do that after the OTA init as it will set hostname too. 
  MDNS.begin(status.name.c_str());
  MDNS.disableArduino(); // disable announcement from OTA
  MDNS.addService("http", "tcp", 80);
  
  // http Webserver for receiving commands
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

  // http "config" API to query persistent configuration
  httpServer.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    // send global configuration Params
    size_t len = 4096;
    char buffer[len];
    size_t written = writeAllParamsToBuffer(buffer, true);
    Serial.printf("DEBUG /config repsonse bytes %d\n", written );    
    request->send(200, "text/html", buffer);
  });

  // websocket "set" API to set and apply values
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len) {
        if (info->opcode == WS_TEXT) {
          data[len] = 0;
          setFromWs((char*)data);
        }
      }
    }
  });
  httpServer.addHandler(&ws);

  // serve index.html single page UI
  httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  httpServer.begin();
  Serial.println("Webserver started.");

  // start ArtnetNode
  artnetnode.setArtDmxCallback([](uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
    for(uint8_t i=0; i<axes.size(); i++)
      if(axes[i].ik_dmx_ch && axes[i].ik_dmx_ch <= length)
        axes[i].ik_ext_in = axes[i].ik_dmx_a / 255.f * (float)data[axes[i].ik_dmx_ch-1];
  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");

  // start OSC Udp receiver
  udp.begin(oscInPort);
  Serial.println("OSC receiver started.");
  
  IPAddress osc_out_ip;
  osc_out_ip.fromString(status.osc_out_ip.c_str());
  oscUdp.setDestination(osc_out_ip, status.osc_out_port);
  Serial.println("OSC sender started.");
 
  vTaskPrioritySet(NULL, 2);
 
  // start motion task with priority 11 - the web server defaults to 10, so we are higher. 
  xTaskCreatePinnedToCore(motionLoop,"MotionLoop",4096,NULL,19,NULL ,1); 
 
  // light status led
  digitalWrite(statusLedPin,HIGH);
}

void oscMessageParser( MicroOscMessage& receivedOscMessage) {
  Serial.printf("OSC in : %s",receivedOscMessage.buffer);

  if ( receivedOscMessage.checkOscAddress("/midi") ) {
    const uint8_t* midi;
    receivedOscMessage.nextAsMidi(&midi);    
    uint8_t type = midi[0] & 0xF0;
    Serial.printf("MIDI cmd/channel:%d d1:%d d2:%d ",midi[0],midi[1],midi[2]);
    if(type == 0x90) {// note on
      midi_picker.pick(midi[1]);
    }
    if(type == 0xB0) {// CC change
      for(uint8_t i=0; i<axes.size(); i++)
        if(axes[i].ik_midi_cc == midi[1])
          axes[i].ik_ext_in = axes[i].ik_midi_a / 128.f * midi[2];
    }
    Serial.println();
  }
  if ( receivedOscMessage.checkOscAddress("/xyza") ) {
    uint8_t axe_idxs[] = {0,1,2,6}; // x,y,z,a
    for(uint8_t i=0; i<4; i++)
      axes[axe_idxs[i]].ik_ext_in = receivedOscMessage.nextAsFloat();
  }
}

void loop() 
{
  ArduinoOTA.handle();
  if(WiFi.getMode() == WIFI_MODE_AP)
    dnsServer.processNextRequest();
  artnetnode.read();
  oscUdp.onOscMessageReceived( oscMessageParser );
  ws.cleanupClients();
  
  for(uint8_t i=0; i<cues.size(); i++)
    cues[i].update([](char* cmd)->void{ setFromWs(cmd, true); });

  status.vbus = analogRead(36) / 4096.f * 3.3f * status.voltage_divider;

  if(status.send_status) {
    // status.nvs_free = prefs.freeEntries();
    status.fs_free  = LittleFS.totalBytes() - LittleFS.usedBytes();
    status.ram_free = esp_get_minimum_free_heap_size();
    status.send_status = 0;
    send_status();
  }
  if(status.reset)    
    ESP.restart();

  delay(10);
}
