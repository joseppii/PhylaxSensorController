
#include "odometer.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

char base_link[] = "base_link";
char odom[]      = "odom";

nav_msgs::Odometry odom_msg;
ros::Publisher odometry_pub("/odom", &odom_msg);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster odom_broadcaster;
    
Odometer::Odometer(PololuEncoder *encoder):_encoder(encoder){}

void Odometer::init(ros::NodeHandle &nh, float track)
{
  _track = track;
  //Initialize pololu encoder(s)
  _encoder->init();

  //advertise topics
  nh.advertise(odometry_pub);
  odom_broadcaster.init(nh);
}

void Odometer::updateEncoder()
{
  uint32_t encoder1Count;
  uint32_t encoder2Count;

  _encoder->update(encoder1Count, encoder2Count);

  int32_t dEncoder1 = (encoder1Count - _encoder1CountPrev);
  int32_t dEncoder2 = (encoder2Count - _encoder2CountPrev);

  //update the angle increment in radians
  float dphi1 = ((float)dEncoder1 * _encoder->_enc2rad);
  float dphi2 = ((float)dEncoder2 * _encoder->_enc2rad);
  
  //for encoder index and motor position switching (Right is 1, Left is 2)
  _dPhiR = dphi1;
  _dPhiL = dphi2;
  
  _encoder1CountPrev = encoder1Count;
  _encoder2CountPrev = encoder2Count;
}

void Odometer::evaluateRobotPose(unsigned long diff_time)
{
  float dTh = _encoder->_wheelRadius/(_track) *(_dPhiR - _dPhiL);
  float dist = _encoder->_wheelRadius *(_dPhiR + _dPhiL) / 2;
  float dx = _encoder->_wheelRadius/2 * (cos(_th)*_dPhiR + cos(_th)*_dPhiL);
  float dy = _encoder->_wheelRadius/2 * (sin(_th)*_dPhiR + sin(_th)*_dPhiL);
  long dt = float(diff_time)/1000;
  
  _th+= dTh;
  _x+=dx;
  _y+=dy;
  _vx = dist/dt;
  _vTh = dTh/dt;
  _pathDistance = _pathDistance + sqrt(dx*dx + dy*dy);

  Serial.println("Math stuff = "+String((int32_t)_pathDistance));
}

void Odometer::publish_odom(ros::Time current_time) 
{
  odom_msg.header.stamp          = current_time;
  odom_msg.header.frame_id       = odom;
  odom_msg.child_frame_id        = base_link;
   
  odom_msg.pose.pose.position.x  = _x;
  odom_msg.pose.pose.position.y  = _y;
  odom_msg.pose.pose.position.z  = 0.0;
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(_th);

  odom_msg.twist.twist.linear.x  = _vx;
  odom_msg.twist.twist.linear.y  = 0;
  odom_msg.twist.twist.angular.z = _vTh;

  odometry_pub.publish(&odom_msg);
}

void Odometer::broadcastTf(ros::Time current_time) 
{
  t.header.stamp            = current_time;
  t.header.frame_id         = odom;
  t.child_frame_id          = base_link;

  t.transform.translation.x = _x;
  t.transform.translation.y = _y;
  t.transform.translation.z = 0.0;
  t.transform.rotation      = tf::createQuaternionFromYaw(-_th);

  odom_broadcaster.sendTransform(t);
}
  
