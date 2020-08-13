#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include <ESP32Encoder.h>

class PololuEncoder
{
  public:
    uint32_t _encoder1CountPrev;
    uint32_t _encoder2CountPrev;
    float _dPhiL;
    float _dPhiR;

    float _th;
    float _x;
    float _y;
    float _pathDistance;
    
    PololuEncoder(int gearing, int cpr, float radius);
    
    void init();
    void update(uint32_t &encoder1Count, uint32_t& encoder2Count);
    
  //private:
    float _gears; 
    float _cpr; 
    float _wheelRadius;
    
    float _enc2rev;
    float _enc2rad;
    float _enc2wheel;

    ESP32Encoder *_left_encoder;
    ESP32Encoder *_right_encoder;
};

#endif
