#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <ros.h>
#include "encoder.h"

class Odometer {
  public:
    ros::NodeHandle nh;
    
    Odometer(PololuEncoder *encoder);
    void init(ros::NodeHandle &nh, float track);
    void updateEncoder();
    void evaluateRobotPose(unsigned long diff_time);
    void publish_odom(ros::Time current_time);
    void broadcastTf(ros::Time current_time);
    
  private:
    PololuEncoder *_encoder;
    ros::NodeHandle _nh;

    uint32_t _encoder1CountPrev;
    uint32_t _encoder2CountPrev;

    float _track; 
    float _dPhiL;
    float _dPhiR;

    float _th;
    float _x;
    float _y;
    float _vx;
    float _vTh;
    float _pathDistance;    
};

#endif /* ODOMETRY_H */
