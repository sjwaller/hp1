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

  Vision inst = *this;

  // Register img Subscribers
  image_sub = it.subscribe("/camera/image_raw", 1, &Vision::callback, &inst);
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

