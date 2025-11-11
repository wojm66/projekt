#pragma once
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

struct HivePayload{
    uint8_t id;
    uint8_t active;
      HiveData local;
};


struct MeteoPayload{
  int16_t t_bme;
  int16_t h_bme;
  int16_t p_bme;
  int16_t a_bme;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};


class HiveFrame{
    public:
    uint16_t header;
    union{
      HivePayload hive;
      MeteoPayload meteo;
     } payload;

    HiveFrame(DeviceData& rd);
    HiveFrame(MeteoPayload& bd);
    HiveFrame(uint8_t* buf, int len);
    HiveFrame();
    void Send();
    bool isHive();
    bool isMeteo();
    HivePayload getHiveData();
    MeteoPayload getMeteoData();
    void print();
};


