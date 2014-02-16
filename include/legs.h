#ifndef LEGS_H
#define	LEGS_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <hp1/robot.h>
class Legs
{
public:
    Legs();

    virtual ~Legs();

    void process();

    void publish();

    void update();

    void setup();

private:
	
    ros::NodeHandle n;

    ros::Subscriber robot_sub;

    void callback(const hp1::robot::ConstPtr& robot);

    // Register Leg Publishers
    ros::Publisher rr_coxa_pub, rr_tibia_pub, rr_femur_pub;
    ros::Publisher lr_coxa_pub, lr_tibia_pub, lr_femur_pub;
    ros::Publisher rm_coxa_pub, rm_tibia_pub, rm_femur_pub;
    ros::Publisher lm_coxa_pub, lm_tibia_pub, lm_femur_pub;
    ros::Publisher rf_coxa_pub, rf_tibia_pub, rf_femur_pub;
    ros::Publisher lf_coxa_pub, lf_tibia_pub, lf_femur_pub;

    // Register Leg Messages
    std_msgs::Float64 rr_coxa_msg, rr_tibia_msg, rr_femur_msg;
    std_msgs::Float64 lr_coxa_msg, lr_tibia_msg, lr_femur_msg;
    std_msgs::Float64 rm_coxa_msg, rm_tibia_msg, rm_femur_msg;
    std_msgs::Float64 lm_coxa_msg, lm_tibia_msg, lm_femur_msg;
    std_msgs::Float64 rf_coxa_msg, rf_tibia_msg, rf_femur_msg;
    std_msgs::Float64 lf_coxa_msg, lf_tibia_msg, lf_femur_msg;

    int radToServo(float rads);
	double tickToRad(int tick);
};

#endif	/* LEGS_H */

