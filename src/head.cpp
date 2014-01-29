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
  ros::Rate loop_rate(12);

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
  // Register Joint Publisher
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Configure Joint States
  joint_state.name.resize(2);
  joint_state.position.resize(2);

  joint_state.name[0] = "pan_joint";
  joint_state.name[1] = "tilt_joint";

  // Register Head Publishers
  pan_pub = n.advertise<std_msgs::Float64>("pan_controller/command", 1);
  tilt_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1);

  Head inst = *this;

  // Register Head Subscribers
  pan_sub = n.subscribe("pan_controller/state", 1000, &Head::listen, &inst);
  tilt_sub = n.subscribe("tilt_controller/state", 1000, &Head::listen, &inst);
}

void Head::setup()
{

}

void Head::update()
{

}

void Head::process()
{
  // Set Leg Message Data
  pan_msg.data = pan_angle;
  tilt_msg.data = tilt_angle;

  // Set Joint State Positions
  joint_state.position[0] = pan_angle;
  joint_state.position[1] = tilt_angle;
}

void Head::publish()
{
  // Publish Head Message Data
  pan_pub.publish(pan_msg);
  tilt_pub.publish(tilt_msg);

  // Publish Joint State Positions
  joint_state.header.stamp = ros::Time::now();
  joint_pub.publish(joint_state);
}

void Head::listen(const dynamixel_msgs::JointState::ConstPtr& msg)
{
  /*
    Header header
    string name         # joint name
    int32[] motor_ids   # motor ids controlling this joint
    int32[] motor_temps # motor temperatures, same order as motor_ids

    float64 goal_pos    # commanded position (in radians)
    float64 current_pos # current joint position (in radians)
    float64 error       # error between commanded and current positions (in radians)
    float64 velocity    # current joint speed (in radians per second)
    float64 load        # current load
    bool is_moving      # is joint currently in motion
   */

  for (int i = 0; i < 2; i++)
  {
    if (joint_state.name[i] == msg->name)
    {
      joint_state.position[i] = msg->current_pos;
    }
  }
}

Head::~Head()
{
}

