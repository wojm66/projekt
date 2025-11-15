#include <Arduino.h>
#include "RemoteNode.h"

RemoteNode::RemoteNode(uint8_t id_, uint8_t active_, int16_t temp_, int16_t hum_, unsigned long ts_)
    : id(id_), active(active_), temperature(temp_), humidity(hum_), timestamp(ts_) {};
RemoteNode::RemoteNode()
:id(0), active(0), temperature(0), humidity(0), timestamp(0)
{};