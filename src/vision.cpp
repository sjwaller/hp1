#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <vision.h>

using namespace std;

int main(int argc, char **argv)
{
  // Initialise ROS	
  ros::init(argc, argv, "vision");

  // Initialise robot	
  Vision *node = new Vision;

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

Vision::Vision()
{
  // Register ROI Publisher
  roi_pub = n.advertise<sensor_msgs::RegionOfInterest>("roi", 1);

  // Register img Subscribers
      // Register Image Transport
  image_transport::ImageTransport it(n);

//  image_transport::TransportHints hints("compressed", ros::TransportHints());

  image_sub = it.subscribe("/camera/image_raw", 1, &Vision::callback);
}

void Vision::setup()
{

}

void Vision::update()
{

}

void Vision::process()
{

}

void Vision::publish()
{

}

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // cv_bridge::CvImageConstPtr cv_ptr;
  // try
  // {
  //   cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // Process cv_ptr->image using OpenCV

}

void Vision::convert_image()
{

}

void Vision::do_camshift()
{

}

void Vision::hue_histogram_as_image()
{

}

void Vision::is_rect_nonzero()
{

}


Vision::~Vision()
{
}

