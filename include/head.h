#ifndef HEAD_H
#define	HEAD_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/RegionOfInterest.h>

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

    // Register Vision Subscribers
    ros::Subscriber roi_sub, robot_angle_sub, state_sub;

    // Register Head Publishers
    ros::Publisher pan_pub, tilt_pub;
    
    // Register Head Messages
    std_msgs::Float64 pan_msg, tilt_msg;
    
    // Helper functions
    int radToServo(float rads);
    double tickToRad(int tick);

    // Define callbacks
    void roiCallback(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void angleCallback(const std_msgs::Float32::ConstPtr& msg);
    void stateCallback(const std_msgs::UInt8::ConstPtr& msg);
};

#endif	/* HEAD_H */

