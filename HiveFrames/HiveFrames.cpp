#include <Arduino.h>
#include "HiveFrames.h"

    HiveFrame::HiveFrame(){
        header=0;
    }
   HiveFrame::HiveFrame(DeviceData& rd)
   {
    header=0xAA55;
    payload.hive.id=rd.id;
    payload.hive.active=rd.active;
    payload.hive.local.temperature= rd.temperature;
    payload.hive.local.humidity= rd.humidity;
   }

   void HiveFrame::Send(){
    uint16_t size=0;
    switch (header){
        case 0xAA55:
        size=sizeof(header)+sizeof(payload.hive);
        break;
        case 0xBB66:
        size=sizeof(header)+sizeof(payload.meteo);
        break;
        default:
        return;
    }
    Serial.write((uint8_t*)this, size);  
   }
 

   HiveFrame::HiveFrame(MeteoPayload& bd)
   {
    header=0xBB66;
    payload.meteo=bd;

   }

   HiveFrame::HiveFrame(uint8_t* buf, int len){
    memcpy((void*)this, buf,len>sizeof(HiveFrame)?sizeof(HiveFrame):len);
   }

   bool HiveFrame::isHive(){
    if(header==0xAA55) return true;
    return false;
   }
    bool HiveFrame::isMeteo(){
        if(header==0xBB66) return true;
    return false;
    }
    HivePayload HiveFrame::getHiveData(){
        return payload.hive;
    }
    MeteoPayload HiveFrame::getMeteoData(){
        return payload.meteo;
    }

    void HiveFrame::print(){
        Serial.printf("header: %.4x\n",header);
        if(header=0xAA55){
            Serial.printf("hive %d\n", payload.hive.id);
            Serial.printf("hive %d\n",payload.hive.active);
            Serial.printf("hive %d\n",payload.hive.local.temperature);
            Serial.printf("hive %d\n",payload.hive.local.humidity);
        }
        if(header=0xBB66){
            Serial.printf("meteo %d\n",payload.meteo.t_bme);
            Serial.printf("meteo %d\n",payload.meteo.h_bme);
            Serial.printf("meteo %d\n",payload.meteo.p_bme);
            Serial.printf("meteo %d\n",payload.meteo.a_bme);
            Serial.printf("meteo %d\n",payload.meteo.pm1_0);
            Serial.printf("meteo %d\n",payload.meteo.pm2_5);
            Serial.printf("meteo %d\n",payload.meteo.pm10);
        }
    }