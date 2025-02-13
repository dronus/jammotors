// drive Xiaomi Cybergear geared brushless servo motor by CAN bus

#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

// pins and ids for Cybergear CAN bus ("TWAI" in ESP32)
const uint8_t RX_PIN = 4;
const uint8_t TX_PIN = 5;
const uint8_t MASTER_CAN_ID = 0x00;

void check_alerts();

struct DriverCybergear;
DriverCybergear* can_handlers[max_channels];
struct DriverCybergear : public Driver{
  XiaomiCyberGearDriver cybergear;
  uint8_t can_id;
  uint32_t last_torque_limit;
  float torque;

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
    if(c.enabled && !c.last_enabled) {
      cybergear.init_motor(MODE_MOTION);
      cybergear.set_limit_torque(torque);
      cybergear.enable_motor();
    }

    // write new can_id if requested to
    if(c.set_can_id > 0) {
      Serial.printf("Set new CAN ID %d for motor on CAN ID %d\n", c.set_can_id, can_id);
      cybergear.set_motor_can_id(c.set_can_id);
      c.set_can_id = 0;
    }
      
    // reset zero origin if prompted to
    if(c.reset_zero) {
      cybergear.set_mech_position_to_zero();
      c.reset_zero = false;
    }
    torque = c.enabled ? torque + max(min(c.accel/1000.f-torque, 0.01f),-0.01f) : 0.f;
    if(torque != last_torque_limit) {
      cybergear.set_limit_torque(torque);
      last_torque_limit = c.accel;
    }

    // execute move
    if(c.enabled)
      cybergear.send_motion_control({c.target/1000.f,0.f,0.f,c.pos_kp/1000.f,c.pos_kd/1000.f});

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
};


// receive incoming CAN messages from CyberGear motor and forward them to driver
void handle_rx_message(twai_message_t& message) {
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

