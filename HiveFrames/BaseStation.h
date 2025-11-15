#pragma once
struct BaseStation {
  int16_t temperature;
  int16_t humidity;
  int16_t pressure;
  int16_t altitude;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};