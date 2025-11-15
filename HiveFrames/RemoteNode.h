#pragma once
struct RemoteNode {
  uint8_t id;
  uint8_t active;
  int16_t temperature;
  int16_t humidity;
  unsigned long timestamp;
  RemoteNode(uint8_t id_, uint8_t active_, int16_t temp_, int16_t hum_, unsigned long ts_);
  RemoteNode();
};
