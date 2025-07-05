// drive PPM-controlled RC servo

#include <WiFiUdp.h>

struct DriverOSC : public Driver {

  MicroOscUdp<128> oscUdp;

  DriverOSC(WiFiUDP& _udp) : oscUdp(&_udp){};

  // virtual ~DriverOSC() {}

  void init(Channel& c) {
    IPAddress osc_out_ip;
    osc_out_ip.fromString(c.osc_out_ip.c_str());
    oscUdp.setDestination(osc_out_ip, c.pin_id);
    Serial.println("OSC remote output started.");
  };

  void update(Channel& c, uint32_t dt) {
    if(c.enabled) {
      // send OSC
      oscUdp.sendMessage("/abcd","ffff",c.target,0,0,0);
      c.position = c.target; // real feedback would need an reverse OSC sender configuration too. 
    }
  };
};

