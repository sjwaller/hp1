#ifndef VISION_H
#define	VISION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;


/*
 *
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * 
 */

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

    // Register Publishers
    ros::Publisher robot_angle_pub;

    ros::Publisher roi_pub;

    /* State is published according to the following enum
     * 0: Waiting for destination
     * 1: Destination tracking enabled
     * 2: Destination Lost
     * 3..255: TBD
     */
    ros::Publisher state_pub;

    // Register Subscribers
    image_transport::Subscriber image_sub;

    ros::Subscriber camera_info_sub;

    ros::Subscriber dest_coord_sub;

    // Declare Variables
    std_msgs::UInt8 state;
    int trackObject;
    bool paused;

    Rect selection;

    int vmin;
    int vmax;
    int smin;

    Mat image;
    int imgWidth;
    int imgHeight;

    // Define callbacks
    void camInfoCallback(const sensor_msgs::CameraInfo & camInfoMsg);
    void destCoordCallback(const sensor_msgs::RegionOfInterest& destROI);
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
  
    void trackArea(Rect window);
    void calcAngle(Point2f destCentre);
    void camShift(Mat inImg);
  
};

#endif	/* VISION_H */

