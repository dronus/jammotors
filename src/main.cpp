
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h>
#include <ArtnetnodeWifi.h>
#include <Preferences.h>
#include "param.h"
#include "FastAccelStepper.h"
#include <ESP32Servo.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"


#define statusLedPin 2

// pins and ids for Cybergear CAN bus ("TWAI" in ESP32)
#define RX_PIN 4
#define TX_PIN 5
uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;

const float pi = 3.1415926f;

Preferences prefs;
ArtnetnodeWifi artnetnode;
AsyncWebServer *httpServer;
DNSServer dnsServer;


#include <WiFiUdp.h>
WiFiUDP udpIn;
unsigned int oscInPort = 8888;
MicroOscUdp<1024> oscReceiver(&udpIn, IPAddress(0,0,0,0), 0);

FastAccelStepperEngine engine = FastAccelStepperEngine();

int homing = 0;

const uint8_t max_channels = 4;

struct Channel;

struct Driver {
  virtual void init(Channel& c) = 0;
  virtual void update(Channel& c, uint32_t dt) = 0;
  virtual ~Driver(){};
};

Driver* createDriver(uint8_t driver_id,uint8_t pin_id);

struct Channel : public Params {

  P_uint8_t (driver_id, 0, 0xFF, 0);
  P_uint8_t (pin_id, 0, 0xFF, 0);
  P_int32_t (poweron_en, 0, 1, 0);
  P_int32_t (speed, 0, 100000, 10000);
  P_int32_t (accel, 0, 100000, 10000);
  P_int32_t (pos_kp, 0, 500000, 1000);
  P_int32_t (pos_kd, 0, 5000, 100);
  P_int32_t (dmx_channel, 0, 255, 0);
  P_int32_t (scale, 0, 10000,  0);
  P_int32_t (offset, -100000, 100000,  0);
  P_int32_t (osc_f, 0, 10000,  1000);
  P_int32_t (osc_a, 0, 100000, 0);
  P_int32_t (osc_p, 0, 100000, 0);
  P_int32_t (random_d, 0, 100000, 1000);
  P_int32_t (random_rd,0, 100000, 1000);
  P_int32_t (random_a ,0, 200000, 0);
  P_int32_t (ik_a ,-100000, 100000, 0);
  P_end;

  int enabled, last_enabled=false;
  int have_alarm;
  int32_t target = 0;
  int32_t position = 0, torque = 0, temperature = 0;
  int32_t manual_target=0;
  int32_t artnet_target=0;
  float ik_target;
  float osc_phase = 0;
  int32_t random_countdown = 0;
  int32_t random_target = 0;
  uint8_t last_driver_id=0, last_pin_id=0;
  Driver* driver=NULL;

  void init() {
    enabled = poweron_en;
  }
  
  void update(uint32_t dt) {

    // check if driver is still up-to-date 
    // and reinitialize if needed
    if(driver_id != last_driver_id || pin_id != last_pin_id) {
      if(driver) delete driver;
      driver = createDriver(driver_id,pin_id);
      last_driver_id = driver_id;
      last_pin_id = pin_id;
      if(driver) driver->init(*this);
    }

    // compute and set motion target
    // set manual target (temporary offset) and stored offset.
    target = manual_target + offset;

    // add artnet commanded target
    target += artnet_target;
   
    // add oscillatory movement 
    osc_phase += osc_f * dt * 2.f * (float)PI / 1000.f / 1000.f;
    osc_phase = fmod(osc_phase, ((float)PI * 2.f));
    target += floor( osc_a * sin(osc_phase + osc_p * 2.f * (float)PI / 1000.f) );
    
    // add random movement
    random_countdown -= dt;
    if(random_countdown < 0) {
      random_target    = random(random_a);
      random_countdown = random_d + random(random_rd);
    }
    target += random_target;
    
    // add IK movement
    if(ik_a != 0)
      target += ik_a * ik_target;

    if(driver)
      driver->update(*this, dt);
    
    last_enabled = enabled;
  }
};

void check_alerts();

struct DriverCybergear;
DriverCybergear* can_handlers[max_channels];
struct DriverCybergear : public Driver{
  XiaomiCyberGearDriver cybergear;
  uint8_t can_id;

  DriverCybergear(uint8_t _can_id) : cybergear(_can_id,MASTER_CAN_ID), can_id(_can_id){
    for(uint8_t i=0; i<max_channels; i++)
      if(!can_handlers[i]) {
        can_handlers[i]=this;
        break;
      }
  }
  virtual ~DriverCybergear() {cybergear.stop_motor();};

  void init(Channel& c) {
    // initialize CyberGear on CAN bus
    cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
    cybergear.init_motor(MODE_MOTION);
    cybergear.set_position_ref(0.0);
    Serial.println("Cybergear started.");
  };

  void update(Channel& c, uint32_t dt) {
    // update enable state
    if(!c.enabled && c.last_enabled)
      cybergear.stop_motor();
    if(c.enabled && !c.last_enabled)
      cybergear.enable_motor();
    // execute move
    if(c.enabled) {
      // cybergear.set_limit_speed(c.speed/1000.f);
      // cybergear.set_position_kp(c.pos_kp/1000.f);
      cybergear.set_limit_torque(c.accel/1000.f);
      // cybergear.set_position_ref(c.target/1000.f);
      cybergear.send_motion_control({c.target/1000.f,0.f,0.f,c.pos_kp/1000.f,c.pos_kd/1000.f});
    }

    // check motor state (handle messages for all CyberGear motors)
    // also updates motor status data that can be relayed to wifi clients then.
    cybergear.request_status();
    check_alerts();

    XiaomiCyberGearStatus status = cybergear.get_status();
    // Serial.printf("POS:%f V:%f T:%f temp:%d\n", status.position, status.speed, status.torque, status.temperature);

    c.position = (int32_t)round(status.position*1000.f);
    c.temperature = status.temperature;
    c.torque      = status.torque * 1000.f;
    // response->printf("%s %d\n","position",(int32_t)round(status.position*1000.f));
    // response->printf("%s %d\n","torque",(int32_t)round(status.torque*1000.f));
    // response->printf("%s %d\n","temperature",(int32_t)status.temperature);
  };
  
  void writeId(uint8_t new_can_id) {
    Serial.printf("Set new CAN ID %d for motor on CAN ID %d\n", new_can_id, can_id);
    cybergear.set_motor_can_id(new_can_id);
  }
};

struct DriverServo : public Driver {
  Servo servo;
  int pin;

  DriverServo(int _pin){ pin=_pin; };
  virtual ~DriverServo() { servo.detach(); }

  void init(Channel& c) {
    servo.setPeriodHertz(50);// Standard 50hz servo
    Serial.println("Servo started.");
  };

  void update(Channel& c, uint32_t dt) {
    if(!c.enabled && c.last_enabled)
      servo.detach();
    if(c.enabled && !c.last_enabled)
      servo.attach(pin, 500, 2400);

    if(c.enabled) {
      servo.write(1000 + max(0,c.target));
      c.position = c.target; // no real feedback possible
    }
  };
};

struct DriverPWM : public Driver {
  ESP32PWM pwm;
  int pin;

  DriverPWM(int _pin){ pin=_pin; };
  virtual ~DriverPWM() { analogWrite(pin,0);}

  void init(Channel& c) {
    analogWriteResolution(10);
    Serial.println("PWM started.");
  };

  void update(Channel& c, uint32_t dt) {
    analogWrite(pin, max(0, min(1023,c.target)));
    c.position = c.target; // no real feedback possible
  };
};

struct DriverStepper : public Driver {
  FastAccelStepper *stepper = NULL;

  const uint8_t pinStep = 16, pinDir = 17, pinEnable = 21, pinAlarm = 22;

  virtual ~DriverStepper() {
    if(stepper) {
      stepper->forceStop();
      stepper->detachFromPin();
      delete stepper;
      pinMode(pinEnable,INPUT);
    }
  }

  // simple external pin handler - the FastAccelStepper original one does not like other interrupts.
  static bool setExternalPin(uint8_t pin, uint8_t value) {
    pin = pin & ~PIN_EXTERNAL_FLAG;
    pinMode(pin, OUTPUT);
    bool oldValue = digitalRead(pin);
    digitalWrite(pin, value);
    return oldValue;
  }

  void init(Channel& c) {
    // start stepper driver
    engine.init();
    stepper = engine.stepperConnectToPin(pinStep, DRIVER_MCPWM_PCNT);
    stepper->setDirectionPin( pinDir + PIN_EXTERNAL_FLAG ); // use external pin handler
    engine.setExternalCallForPin(DriverStepper::setExternalPin);
    pinMode(pinEnable, OUTPUT);
    Serial.println("Stepper started.");
  };

  void update(Channel& c, uint32_t dt) {
    digitalWrite(pinEnable, c.enabled);
    stepper->setSpeedInHz(c.speed);
    stepper->setAcceleration(c.accel);
    stepper->moveTo(c.target);
    c.position = stepper->getCurrentPosition();
  };
};

Driver* createDriver(uint8_t driver_id, uint8_t pin_id) {
  Serial.printf("Create driver %d on pin / CAN id %d\n",driver_id,pin_id);
  if(driver_id == 1) return new DriverStepper();
  if(driver_id == 2) return new DriverServo(pin_id);
  if(driver_id == 3) return new DriverPWM(pin_id);
  if(driver_id == 4) return new DriverCybergear(pin_id);

  return NULL;
}

Channel channels[max_channels];

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
// TODO dosn't work stable for now - often no DHCP on this one fails 
// and no IP address is obtained. 
// Maybe because of broad bandwith which collides with every other WiFi?
void switchWifiAp() {
  Serial.println("Switching WiFi to AP mode.");
//  WiFi.softAP(prefs.getString("name","Motor").c_str(), "motorkraft3000");
  WiFi.softAP(prefs.getString("name","Motor").c_str());
  Serial.print(WiFi.softAPIP());
  Serial.println("Starting DNS (Captive Portal)");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// receive incoming CAN messages from CyberGear motor and forward them to driver
static void handle_rx_message(twai_message_t& message) {
  uint8_t can_id = (message.identifier & 0xFF00) >> 8;
  for(DriverCybergear* dcg : can_handlers)
    if (dcg && can_id == dcg->can_id)
      dcg->cybergear.process_message(message);
}

// check alerts from CAN bus, receive incoming messages from CyberGear motor
void check_alerts(){
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

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
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

    readFromRequest(&channels[channel_id], channel_id, request);
    
    // check for global parameters
    readFromRequest(&global_params, -1, request);
    // check for global string parameters
    for(char* param : global_string_params)
      if (request->hasParam(param) )
        prefs.putString(param, request->getParam(param)->value());

    if (request->hasParam("set_can_id") ) {
      uint8_t zahl = request->getParam("set_can_id")->value().toInt();
      static_cast<DriverCybergear*>(channels[channel_id].driver)->writeId(zahl);
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
