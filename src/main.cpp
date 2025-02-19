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
#include "motion.h"
#include "kinematic.h"
#include "driver_stepper.h"
#include "driver_servo.h"
#include "driver_pwm.h"
#include "driver_cybergear.h"
#include "midi_picker.h"



Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer httpServer(80);
AsyncWebSocket ws("/ws");

DNSServer dnsServer;


#include <WiFiUdp.h>
WiFiUDP udp;
unsigned int oscInPort = 8888;
MicroOscUdp<1024> oscUdp(&udp, IPAddress(192,168,0,126), 8888);
MidiPicker midi_picker;
Kinematic kinematic;

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

struct Status : public Params {
  P_float (dt, false, 0, 0, 0);
  P_float (dt_max, false, 0, 0, 0);
  P_float (ik_error, false, 0, 0, 0);
  P_uint8_t (send_status,false,0,1,0);
  P_float(vbus,false,0,1,0);
  P_float(voltage_divider, true, 0, 64000, 10000);
  P_end;
} status;

const uint8_t max_axes = 4;
Axis axes[max_axes];

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
      int val = prefs.getFloat(prefs_name); 
      Serial.print(val);
      p->set(val);
    } else {
      Serial.print(" use default ");
      Serial.print(p->get());
    }
    Serial.println();
  }
}

// set the parameter matching "key" from the given Params struct to "value".
void setParam(Params* params, int16_t channel_id, char* key, char* value_str) {

  for(Param* p = params->getParams(); p ; p = p->next()) {

    char prefs_name[32];
    if(channel_id > -1) // per-channel pref names are suffixed by '_' and the channel_id
      snprintf(prefs_name, sizeof(prefs_name), "%s_%d", p->desc->name, channel_id);
    else
      snprintf(prefs_name, sizeof(prefs_name), "%s", p->desc->name);

    if ( strcmp(prefs_name, key) == 0 ) {
      float value = atof(value_str);
      p->set(value);
      if(p->desc->persist)
        prefs.putFloat(prefs_name, value);
    }
  }
}

// set single parameter from incoming key, value message.
void setFromWs(char* key_value)  {
  Serial.printf("Set %s\n",key_value);
  char* key       = key_value;
  char* value_str = strchr(key_value,' ');
  if(!value_str) return; // no value
  *(value_str++) = 0; // mark end of key and advance
  Serial.printf("Set %s %s\n",key,value_str);

  // handle persistent per-channel parameters
  for(uint8_t channel_id = 0; channel_id < max_channels; channel_id++)
    setParam(&channels[channel_id], channel_id, key, value_str);
  
  // check for global parameters
  setParam(&status, -1,  key, value_str);
  setParam(&kinematic, -1,  key, value_str);
  // check for global string parameters
  for(char* param : global_string_params)
    if (strcmp(key, param) == 0 )
      prefs.putString(param, value_str);
  setParam(&midi_picker, -1,  key, value_str);

  // check for axes parameters
  for(uint8_t i=0; i<max_axes; i++)
    setParam(&axes[i], i,  key, value_str);

  // TODO handle instantaneous commands
  //if (request->hasParam("reset") )
  //  ESP.restart();
};

void writeParamsToBuffer(char*& ptr, Params& params, bool persistent, int8_t index=-1) {
  for(Param* p = params.getParams(); p; p = p->next())
    if(p->desc->persist == persistent)
      if(index == -1)
        ptr += (size_t)sprintf(ptr,"%s %.5g\n",p->desc->name, p->get());
      else
        ptr += (size_t)sprintf(ptr,"%s_%d %.5g\n",p->desc->name, index, p->get());
}

template <typename Type> void writeParamsArrayToBuffer (char*& ptr, Type params[], uint8_t len,  bool persistent) {
  for(int8_t i=0; i<len; i++)
    writeParamsToBuffer(ptr, params[i], persistent, i );
}

void writeAllParamsToBuffer(char* buffer, bool persistent) {
  char* ptr = buffer;
  writeParamsArrayToBuffer(ptr, channels, max_channels, persistent);
  writeParamsToBuffer(ptr, kinematic, persistent);
  writeParamsToBuffer(ptr, status, persistent);
  writeParamsToBuffer(ptr, midi_picker, persistent);
  writeParamsArrayToBuffer(ptr, axes, max_axes, persistent);
}

// WebSocket "status" API to send current status
void send_status() {
  size_t len = 4096;
  char buffer[len];
  writeAllParamsToBuffer(buffer, false);
  ws.textAll(buffer);
}

uint32_t last_time;
const uint32_t cycle_time = 10;
void motionLoop(void* dummy){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil( &xLastWakeTime, cycle_time );
    // compute delta time
    uint32_t time = millis();
    uint32_t dt = time - last_time;
    if(last_time == 0) dt=1;
    last_time = time;
    status.dt = dt / 1000.f;
    status.dt_max = max(status.dt, status.dt_max);

    for(Axis& axis : axes)
      axis.ik_input = 0;
    midi_picker.update(axes,status.dt);
    kinematic.update(status.dt, axes, channels);
    status.ik_error = kinematic.update_feedback(channels, axes); // to get IK error

    for(Channel& c : channels)
      c.update(status.dt);
     
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

  // initialize LittleFS on flash for persistent parameter storage by Prefs library
  bool spiffsBeginSuccess = LittleFS.begin();
  if (!spiffsBeginSuccess)
    Serial.println("LittleFS cannot be mounted.");
  else
    Serial.println("LittleFS started.");

  prefs.begin("motor");
  Serial.println("Prefs started.");

  readPrefs(&kinematic);
  readPrefs(&midi_picker);

  for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
    Channel& channel = channels[channel_id];
    // read preferences
    readPrefs(&channel,channel_id);
    // initialize
    channel.init();
  }
  for(uint8_t axis_id = 0; axis_id<max_axes; axis_id++)
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
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, Content-Type, Access-Control-Request-Method, Access-Control-Request-Headers");

  // http "config" API to query persistent configuration
  httpServer.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    // send global configuration Params
    size_t len = 4096;
    char buffer[len];
    writeAllParamsToBuffer(buffer, true);
    response->print(buffer);
    // send global string configuration
    for(char* param : global_string_params)
      response->printf("%s %s\n",param, prefs.getString(param,"").c_str());
    request->send(response);
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
    for(uint8_t channel_id = 0; channel_id<max_channels; channel_id++) {
      Channel& c = channels[channel_id];
      if(c.dmx_channel > 0 && c.dmx_channel-1 < length)
        c.artnet_target = c.scale * ((uint32_t)data[c.dmx_channel-1]);
    }
    
    for(uint8_t i=0; i<max_axes; i++)
      if(axes[i].ik_dmx_ch && axes[i].ik_dmx_ch <= length)
        axes[i].ik_dmx_target = axes[i].ik_dmx_a / 255.f * (float)data[axes[i].ik_dmx_ch-1];

  });
  artnetnode.begin();
  Serial.println("ArtNet node started.");

  // start OSC Udp receiver
  udp.begin(oscInPort);
  Serial.println("OSC receiver started.");
 
  vTaskPrioritySet(NULL, 2);
 
  // start motion task
  
  xTaskCreatePinnedToCore(
                    motionLoop,
                    "MotionLoop",          /* Text name for the task. */
                    4096,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    6,/* Priority at which the task is created. */
                    NULL ,1); 
 
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
      for(uint8_t i=0; i<max_axes; i++)
        if(axes[i].ik_midi_cc == midi[1])
          axes[i].ik_midi_target = axes[i].ik_midi_a / 128.f * midi[2];
    }
    Serial.println();
  }
}

void loop() 
{
  ArduinoOTA.handle();
  if(WiFi.getMode() == WIFI_MODE_AP)
    dnsServer.processNextRequest();
  artnetnode.read();
  oscUdp.onOscMessageReceived( oscMessageParser );

  for(uint8_t i=0; i<max_axes; i++)
    if(axes[i].ik_midi_cc) {
      float pos = (axes[i].ik_feedback - axes[i].ik_offset) / (float)axes[i].ik_midi_a * 128.f;
      pos = max(0.f,min(127.f,pos));
      int32_t midi_data = (0xB0<<24) + (axes[i].ik_midi_cc<<16) + (((uint8_t)pos)<<8); // - 2**32
      oscUdp.sendInt("/midi",midi_data);
    }

  status.vbus = analogRead(36) / 4096.f * 3.3f * status.voltage_divider;

  if(status.send_status) {
    status.send_status = 0;
    send_status();
  }

  delay(10);
}
