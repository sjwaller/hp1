#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <phoenix.h>
#include <legs.h>

const float DEG2RAD = 0.01745;

using namespace std;

int main(int argc, char **argv)
{
  // Initialise ROS 
  ros::init(argc, argv, "legs");

  // Initialise robot 
  Legs *legs = new Legs;

  // Main loop
  ros::Rate loop_rate(30);

  legs->setup();

  while (ros::ok())
  {
    legs->update();

    legs->process();

    legs->publish();

    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}

Legs::Legs()
{
 // Register input subscriber
  robot_sub = n.subscribe<hp1::robot>("hp1/robot", 30, &Legs::callback, this);

  // Register Leg Publishers
  lf_coxa_pub = n.advertise<std_msgs::Float64>("lf_coxa_controller/command", 1);
  lf_femur_pub = n.advertise<std_msgs::Float64>("lf_femur_controller/command", 1);
  lf_tibia_pub = n.advertise<std_msgs::Float64>("lf_tibia_controller/command", 1);

  lm_coxa_pub = n.advertise<std_msgs::Float64>("lm_coxa_controller/command", 1);
  lm_femur_pub = n.advertise<std_msgs::Float64>("lm_femur_controller/command", 1);
  lm_tibia_pub = n.advertise<std_msgs::Float64>("lm_tibia_controller/command", 1);

  lr_coxa_pub = n.advertise<std_msgs::Float64>("lr_coxa_controller/command", 1);
  lr_femur_pub = n.advertise<std_msgs::Float64>("lr_femur_controller/command", 1);
  lr_tibia_pub = n.advertise<std_msgs::Float64>("lr_tibia_controller/command", 1);

  rf_coxa_pub = n.advertise<std_msgs::Float64>("rf_coxa_controller/command", 1);
  rf_femur_pub = n.advertise<std_msgs::Float64>("rf_femur_controller/command", 1);
  rf_tibia_pub = n.advertise<std_msgs::Float64>("rf_tibia_controller/command", 1);
 
  rm_coxa_pub = n.advertise<std_msgs::Float64>("rm_coxa_controller/command", 1);
  rm_femur_pub = n.advertise<std_msgs::Float64>("rm_femur_controller/command", 1);
  rm_tibia_pub = n.advertise<std_msgs::Float64>("rm_tibia_controller/command", 1);

  rr_coxa_pub = n.advertise<std_msgs::Float64>("rr_coxa_controller/command", 1);
  rr_femur_pub = n.advertise<std_msgs::Float64>("rr_femur_controller/command", 1);
  rr_tibia_pub = n.advertise<std_msgs::Float64>("rr_tibia_controller/command", 1);
}

void Legs::setup()
{

}

void Legs::update()
{

}

/* Convert radians to servo position offset. */
int Legs::radToServo(float rads)
{
	float val = (rads*100)/51 * 100;
	return (int) val;
}

/* Convert servo position offset to radians*/
double Legs::tickToRad(int tick)
{
	return ((double)tick-512) * 0.0051;
}


void Legs::callback(const hp1::robot::ConstPtr& robot)
{
  // Set Leg Message Data
  #define cPwmMult      128
  #define cPwmDiv       375  
  #define cPFConst      512 // half of our 1024 range
  
  // radians = tickToRad((((angle))* cPwmMult) / cPwmDiv +cPFConst);

  lf_coxa_msg.data =  tickToRad((((robot->angles[0]))  * cPwmMult) / cPwmDiv + cPFConst);
  lf_femur_msg.data = tickToRad((((robot->angles[1])) * cPwmMult) / cPwmDiv + cPFConst);
  lf_tibia_msg.data = tickToRad((((robot->angles[2])) * cPwmMult) / cPwmDiv + cPFConst);

  lm_coxa_msg.data =  tickToRad((((robot->angles[3]))  * cPwmMult) / cPwmDiv + cPFConst);
  lm_femur_msg.data = tickToRad((((robot->angles[4])) * cPwmMult) / cPwmDiv + cPFConst);
  lm_tibia_msg.data = tickToRad((((robot->angles[5])) * cPwmMult) / cPwmDiv + cPFConst);

  lr_coxa_msg.data =  tickToRad((((robot->angles[6]))  * cPwmMult) / cPwmDiv + cPFConst);
  lr_femur_msg.data = tickToRad((((robot->angles[7])) * cPwmMult) / cPwmDiv + cPFConst);
  lr_tibia_msg.data = tickToRad((((robot->angles[8])) * cPwmMult) / cPwmDiv + cPFConst);

  rf_coxa_msg.data =  tickToRad((((robot->angles[9]))  * cPwmMult) / cPwmDiv + cPFConst);
  rf_femur_msg.data = tickToRad((((robot->angles[10])) * cPwmMult) / cPwmDiv + cPFConst);
  rf_tibia_msg.data = tickToRad((((robot->angles[11])) * cPwmMult) / cPwmDiv + cPFConst);

  rm_coxa_msg.data =  tickToRad((((robot->angles[12]))  * cPwmMult) / cPwmDiv + cPFConst);
  rm_femur_msg.data = tickToRad((((robot->angles[13])) * cPwmMult) / cPwmDiv + cPFConst);
  rm_tibia_msg.data = tickToRad((((robot->angles[14])) * cPwmMult) / cPwmDiv + cPFConst);

  rr_coxa_msg.data =  tickToRad((((robot->angles[15]))  * cPwmMult) / cPwmDiv + cPFConst);
  rr_femur_msg.data = tickToRad((((robot->angles[16])) * cPwmMult) / cPwmDiv + cPFConst);
  rr_tibia_msg.data = tickToRad((((robot->angles[17])) * cPwmMult) / cPwmDiv + cPFConst);
}

void Legs::process()
{
  // // Set Leg Message Data
  // #define cPwmMult      128
  // #define cPwmDiv       375  
  // #define cPFConst      512 // half of our 1024 range
  
  // // radians = tickToRad((((angle))* cPwmMult) / cPwmDiv +cPFConst);

  // lf_coxa_msg.data =  tickToRad((((CoxaAngle1[5]))  * cPwmMult) / cPwmDiv + cPFConst);
  // lf_femur_msg.data = tickToRad((((FemurAngle1[5])) * cPwmMult) / cPwmDiv + cPFConst);
  // lf_tibia_msg.data = tickToRad((((TibiaAngle1[5])) * cPwmMult) / cPwmDiv + cPFConst);

  // lm_coxa_msg.data =  tickToRad((((CoxaAngle1[4]))  * cPwmMult) / cPwmDiv + cPFConst);
  // lm_femur_msg.data = tickToRad((((FemurAngle1[4])) * cPwmMult) / cPwmDiv + cPFConst);
  // lm_tibia_msg.data = tickToRad((((TibiaAngle1[4])) * cPwmMult) / cPwmDiv + cPFConst);

  // lr_coxa_msg.data =  tickToRad((((CoxaAngle1[3]))  * cPwmMult) / cPwmDiv + cPFConst);
  // lr_femur_msg.data = tickToRad((((FemurAngle1[3])) * cPwmMult) / cPwmDiv + cPFConst);
  // lr_tibia_msg.data = tickToRad((((TibiaAngle1[3])) * cPwmMult) / cPwmDiv + cPFConst);

  // rf_coxa_msg.data =  tickToRad((((-CoxaAngle1[2]))  * cPwmMult) / cPwmDiv + cPFConst);
  // rf_femur_msg.data = tickToRad((((-FemurAngle1[2])) * cPwmMult) / cPwmDiv + cPFConst);
  // rf_tibia_msg.data = tickToRad((((-TibiaAngle1[2])) * cPwmMult) / cPwmDiv + cPFConst);

  // rm_coxa_msg.data =  tickToRad((((-CoxaAngle1[1]))  * cPwmMult) / cPwmDiv + cPFConst);
  // rm_femur_msg.data = tickToRad((((-FemurAngle1[1])) * cPwmMult) / cPwmDiv + cPFConst);
  // rm_tibia_msg.data = tickToRad((((-TibiaAngle1[1])) * cPwmMult) / cPwmDiv + cPFConst);

  // rr_coxa_msg.data =  tickToRad((((-CoxaAngle1[0]))  * cPwmMult) / cPwmDiv + cPFConst);
  // rr_femur_msg.data = tickToRad((((-FemurAngle1[0])) * cPwmMult) / cPwmDiv + cPFConst);
  // rr_tibia_msg.data = tickToRad((((-TibiaAngle1[0])) * cPwmMult) / cPwmDiv + cPFConst);
}

void Legs::publish()
{
  // Publish Leg Message Data
  lf_coxa_pub.publish(lf_coxa_msg);
  lf_femur_pub.publish(lf_femur_msg);
  lf_tibia_pub.publish(lf_tibia_msg);

  lm_coxa_pub.publish(lm_coxa_msg);
  lm_femur_pub.publish(lm_femur_msg);
  lm_tibia_pub.publish(lm_tibia_msg);

  lr_coxa_pub.publish(lr_coxa_msg);
  lr_femur_pub.publish(lr_femur_msg);
  lr_tibia_pub.publish(lr_tibia_msg);

  rf_coxa_pub.publish(rf_coxa_msg);
  rf_femur_pub.publish(rf_femur_msg);
  rf_tibia_pub.publish(rf_tibia_msg);

  rm_coxa_pub.publish(rm_coxa_msg);
  rm_femur_pub.publish(rm_femur_msg);
  rm_tibia_pub.publish(rm_tibia_msg);

  rr_coxa_pub.publish(rr_coxa_msg);
  rr_femur_pub.publish(rr_femur_msg);
  rr_tibia_pub.publish(rr_tibia_msg);
}

Legs::~Legs()
{
  
}