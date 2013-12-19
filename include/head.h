#ifndef HEAD_H
#define	HEAD_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

const double degree = M_PI / 180;

// Head Angles
double pan_angle, tilt_angle;

class Head
{
public:
    Head();

    virtual ~Head();

    void setup();

    void update();

    void process();

    void publish();

private:
    ros::NodeHandle n;

    // Register Joint Publisher
    ros::Publisher joint_pub;

    // message declarations
    sensor_msgs::JointState joint_state;

    // Register Head Publishers
    ros::Publisher pan_pub, tilt_pub;

    // Register Head Subscribers
    ros::Subscriber pan_sub, tilt_sub;

    // Register Leg Messages
    std_msgs::Float64 pan_msg, tilt_msg;

    void listen(const dynamixel_msgs::JointState::ConstPtr& msg);
};

#endif	/* HEAD_H */

