#ifndef INPUT_H
#define	INPUT_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <hp1/input.h>

class Input
{
public:
    Input();

private:
    ros::NodeHandle n;
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    int scale_linear_,scale_angular_, deadzone_;
    
    ros::Publisher joy_pub;
    
    ros::Subscriber joy_sub;
};

#endif	/* INPUT_H */