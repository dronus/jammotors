
struct Channel;

struct Driver {
  virtual void init(Channel& c) = 0;
  virtual void update(Channel& c, uint32_t dt) = 0;
  virtual ~Driver(){};
};

Driver* createDriver(uint8_t driver_id,uint8_t pin_id);

