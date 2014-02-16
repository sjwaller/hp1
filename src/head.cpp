#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <head.h>

using namespace std;

int main(int argc, char **argv)
{
  // Initialise ROS	
  ros::init(argc, argv, "head");

  // Initialise robot	
  Head *node = new Head;

  // Main loop
  ros::Rate loop_rate(30);

  node->setup();

  while (ros::ok())
  {
    node->update();

    node->process();

    node->publish();

    loop_rate.sleep();
  }

  return 0;
}

Head::Head()
{
  // Register Head Publishers
  pan_pub = n.advertise<std_msgs::Float64>("pan_controller/command", 1);
  tilt_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1);

  // Register Vision Subscribers
  roi_sub             = n.subscribe<sensor_msgs::RegionOfInterest>("hp1/vision_roi", 10, &Head::roiCallback, this);
  robot_angle_sub     = n.subscribe<std_msgs::Float32>("hp1/vision_angle", 10, &Head::angleCallback, this);
  state_sub           = n.subscribe<std_msgs::UInt8>("hp1/vision_state", 10, &Head::stateCallback, this);
}

void Head::setup()
{
  ROS_INFO_STREAM("Head Initialized");
}

void Head::update()
{
  // Calculate angles
  pan_angle = 0;
  tilt_angle = 0;
}

void Head::process()
{
  // Set Head Message Data
  #define cPwmMult      128
  #define cPwmDiv       375  
  #define cPFConst      512 // half of our 1024 range
  
  pan_msg.data =  tickToRad((((pan_angle))  * cPwmMult) / cPwmDiv + cPFConst);
  tilt_msg.data = tickToRad((((tilt_angle)) * cPwmMult) / cPwmDiv + cPFConst);

  // Set Message Data
  pan_msg.data = pan_angle;
  tilt_msg.data = tilt_angle;
}

void Head::publish()
{
  // Publish Head Message Data
  pan_pub.publish(pan_msg);
  tilt_pub.publish(tilt_msg);
}

/* Convert radians to servo position offset. */
int Head::radToServo(float rads)
{
  float val = (rads*100)/51 * 100;
  return (int) val;
}

/* Convert servo position offset to radians*/
double Head::tickToRad(int tick)
{
  return ((double)tick-512) * 0.0051;
}

void Head::roiCallback(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
  ROS_INFO_STREAM("Head roiCallback");
  // msg->current_pos
}

void Head::angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO_STREAM("Head angleCallback");
  // msg->current_pos
}

void Head::stateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  ROS_INFO_STREAM("Head stateCallback");
  // msg->current_pos
}

Head::~Head()
{

}

