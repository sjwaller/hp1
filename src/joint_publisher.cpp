#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <joint_publisher.h>

using namespace std;

int main(int argc, char **argv)
{
  // Initialise ROS	
  ros::init(argc, argv, "joint_publisher");

  // Initialise JointPublisher
  JointPublisher *publisher = new JointPublisher;

  // Main loop
  ros::Rate loop_rate(14);

  while (ros::ok())
  {
    publisher->publish();

    loop_rate.sleep();
  }

  return 0;
}

JointPublisher::JointPublisher()
{
  // Register Joint Publisher
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Configure Joint States
  joint_state.name.resize(14);
  joint_state.position.resize(14);

  joint_state.name[1 - 1] = "lf_coxa_joint";
  joint_state.name[4 - 1] = "lf_femur_joint";
  joint_state.name[6 - 1] = "lf_tibia_joint";

  joint_state.name[7 - 1] = "lr_coxa_joint";
  joint_state.name[10 - 1] = "lr_femur_joint";
  joint_state.name[12 - 1] = "lr_tibia_joint";

  joint_state.name[2 - 1] = "rf_coxa_joint";
  joint_state.name[3 - 1] = "rf_femur_joint";
  joint_state.name[5 - 1] = "rf_tibia_joint";

  joint_state.name[8 - 1] = "rr_coxa_joint";
  joint_state.name[9 - 1] = "rr_femur_joint";
  joint_state.name[11 - 1] = "rr_tibia_joint";

  joint_state.name[12] = "pan_joint";
  joint_state.name[13] = "tilt_joint";

  JointPublisher inst = *this;

  // Register Leg Subscribers
  lf_coxa_sub = n.subscribe("lf_coxa_controller/state", 1000, &JointPublisher::listen, &inst);
  lf_femur_sub = n.subscribe("lf_femur_controller/state", 1000, &JointPublisher::listen, &inst);
  lf_tibia_sub = n.subscribe("lf_tibia_controller/state", 1000, &JointPublisher::listen, &inst);

  lr_coxa_sub = n.subscribe("lr_coxa_controller/state", 1000, &JointPublisher::listen, &inst);
  lr_femur_sub = n.subscribe("lr_femur_controller/state", 1000, &JointPublisher::listen, &inst);
  lr_tibia_sub = n.subscribe("lr_tibia_controller/state", 1000, &JointPublisher::listen, &inst);

  rf_coxa_sub = n.subscribe("rf_coxa_controller/state", 1000, &JointPublisher::listen, &inst);
  rf_femur_sub = n.subscribe("rf_femur_controller/state", 1000, &JointPublisher::listen, &inst);
  rf_tibia_sub = n.subscribe("rf_tibia_controller/state", 1000, &JointPublisher::listen, &inst);

  rr_coxa_sub = n.subscribe("rr_coxa_controller/state", 1000, &JointPublisher::listen, &inst);
  rr_femur_sub = n.subscribe("rr_femur_controller/state", 1000, &JointPublisher::listen, &inst);
  rr_tibia_sub = n.subscribe("rr_tibia_controller/state", 1000, &JointPublisher::listen, &inst);
 
  // Register Head Subscribers
  pan_sub = n.subscribe("pan_controller/state", 1000, &JointPublisher::listen, &inst);
  tilt_sub = n.subscribe("tilt_controller/state", 1000, &JointPublisher::listen, &inst);
}

void JointPublisher::publish()
{
  // Publish Joint State Positions
  joint_state.header.stamp = ros::Time::now();
  joint_pub.publish(joint_state);
}

void JointPublisher::listen(const dynamixel_msgs::JointState::ConstPtr& msg)
{
  for (int i = 0; i < 12; i++)
  {
    if (joint_state.name[i] == msg->name)
    {
      joint_state.position[i] = msg->current_pos;
    }
  }
}

JointPublisher::~JointPublisher()
{

}

