#include "encoder.h"


PololuEncoder::PololuEncoder(int gearing, int cpr, float radius):
  _gears(gearing),
  _cpr(cpr),
  _wheelRadius(radius),
  _dPhiL(0),
  _dPhiR(0),
  _th(0),
  _x(0),
  _y(0),
  _encoder1CountPrev(0),
  _encoder2CountPrev(0),
  _pathDistance(0)
{
  _left_encoder = new ESP32Encoder();
  _right_encoder = new ESP32Encoder(); 
}

void PololuEncoder::init( )
{
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;
  
  _left_encoder->attachFullQuad(19, 18);
  _right_encoder->attachFullQuad(17, 16);

  //Clear encoder count
  _left_encoder->clearCount();
  _right_encoder->clearCount();
  
  _enc2rev = 1.0/1632.0;
  _enc2rad = _enc2rev * 2 * PI;
  _enc2wheel = _enc2rad * _wheelRadius; 
}

void PololuEncoder::update(uint32_t &encoder1Count, uint32_t& encoder2Count)
{
  encoder1Count = _right_encoder->getCount();
  encoder2Count = _right_encoder->getCount();
}
