#ifndef VISION_H
#define	VISION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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

    // Register Image Transport
    image_transport::ImageTransport it;

    // Register Image Subscribers
    image_transport::Subscriber image_sub;

    void callback(const sensor_msgs::ImageConstPtr& msg);

    void convert_image();

    void do_camshift();

    void hue_histogram_as_image();

    void is_rect_nonzero();

};

#endif	/* VISION_H */

