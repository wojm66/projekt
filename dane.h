struct DeviceData {
  uint8_t mac[6];
  uint64_t long_mac;
  int id;
  int16_t temperature;
    uint16_t humidity;
  unsigned long lastSeen;
  bool active;
};

struct HiveData {
    int16_t temperature;
    uint16_t humidity;
};


class Frame{
    public:
    uint16_t header;

    Frame(DeviceData& rd);
    
}