#ifndef JOINT_PUBLISHER_H
#define	JOINT_PUBLISHER_H

#include <ros/ros.h>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

const double degree = M_PI / 180;

class JointPublisher
{
public:
    JointPublisher();

    virtual ~JointPublisher();

    void publish();

private:
    ros::NodeHandle n;

    // Register Joint Publisher
    ros::Publisher joint_pub;

    // message declarations
    sensor_msgs::JointState joint_state;

    // Register Leg Subscribers
    ros::Subscriber rr_coxa_sub, rr_tibia_sub, rr_femur_sub;
    ros::Subscriber lr_coxa_sub, lr_tibia_sub, lr_femur_sub;
    ros::Subscriber rf_coxa_sub, rf_tibia_sub, rf_femur_sub;
    ros::Subscriber lf_coxa_sub, lf_tibia_sub, lf_femur_sub;
  
    // Register Head Subscribers
    ros::Subscriber pan_sub, tilt_sub;
  
    void listen(const dynamixel_msgs::JointState::ConstPtr& msg);
};

#endif	/* JOINT_PUBLISHER_H */

