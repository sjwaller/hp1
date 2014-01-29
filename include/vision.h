#ifndef VISION_H
#define	VISION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Image.h>

class Vision
{
public:
    Vision();

    virtual ~Vision();

    void setup();

    void update();

    void process();

    void publish();

private:
    ros::NodeHandle n;

    // Register ROI Publisher
    ros::Publisher roi_pub;

    // Register Image Subscribers
    ros::Subscriber image_sub;

    void callback(const sensor_msgs::Image::ConstPtr& msg);

    void convert_image(sensor_msgs::Image ros_image);

    void do_camshift();

    void hue_histogram_as_image();

    void is_rect_nonzero();

};

#endif	/* VISION_H */

