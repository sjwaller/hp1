#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <vision.h>

using namespace std;

// https://github.com/ashokzg/ecp/blob/master/src/CamShiftDestTracker.cpp

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
    loop_rate.sleep();
  }

  return 0;
}

Vision::Vision()
{  
  // Register Image Transport
  image_transport::ImageTransport it(n);

  // Register Subscribers
  image_sub           = it.subscribe("camera/image_raw", 1, &Vision::imageCallback, this);
  camera_info_sub     = n.subscribe("camera/camera_info", 1, &Vision::camInfoCallback, this);
  
  // Register ROI Publishers
  roi_pub             = n.advertise<sensor_msgs::RegionOfInterest>("roi", 10);
  robot_angle_pub     = n.advertise<std_msgs::Float32>("robot_angle", 100);
  state_pub           = n.advertise<std_msgs::UInt8>("state", 10);
}

void Vision::setup()
{
  state.data = 0;
  paused = false;

  vmin = 10;
  vmax = 256;
  smin = 30;
}

void Vision::update()
{

}

void Vision::process()
{

}

//This function is called everytime a new image_info message is published
void Vision::camInfoCallback(const sensor_msgs::CameraInfo & camInfoMsg)
{
  //Store the image width for calculation of angle
  imgWidth = camInfoMsg.width;
  imgHeight = camInfoMsg.height;
}

// This function is called everytime a new image is published
void Vision::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //Always copy, returning a mutable CvImage
    //OpenCV expects color images to use BGR channel order.
    cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    //if there is an error during conversion, display it
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Process cv_ptr->image using OpenCV
  camShift(cv_ptr->image);
}

void Vision::camShift(Mat inImg)
{
  static Rect trackWindow;
  static int hsize = 16;
  static float hranges[] = {0,180};
  static const float* phranges = hranges;
  static Mat frame, hsv, hue, mask, hist = Mat::zeros(200, 320, CV_8UC3), backproj;
  RotatedRect trackBox;

  //If the image processing is not paused
  if( !paused )
  {
    //cap >> frame;
    if( inImg.empty() )
    {
      ROS_INFO("Camera image empty");
      return;//break;
    }
  }

  //Use the input image as the reference
  //Only a shallow copy, so relatively fast
  image = inImg;

  if(!paused)
  {
      //Convert the colour space to HSV
      cvtColor(image, hsv, CV_BGR2HSV);

      // If the destination coordinates have been received, then start the tracking
      // trackObject is set when the destination coordinates have been received
      if( trackObject )
      {
          int _vmin = vmin, _vmax = vmax;

          inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                  Scalar(180, 256, MAX(_vmin, _vmax)), mask);
          int ch[] = {0, 0};
          hue.create(hsv.size(), hsv.depth());
          mixChannels(&hsv, 1, &hue, 1, ch, 1);

          // Do the following steps only for the first time
          if( trackObject < 0 )
          {
              // Publish that we have started tracking
              state.data = 1;
              state_pub.publish(state);
              
              // Set the Region of interest and the mask for it
              Mat roi(hue, selection), maskroi(mask, selection);
              
              // Calculate the histogram of this
              calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
              normalize(hist, hist, 0, 255, CV_MINMAX);

              trackWindow = selection;
              trackObject = 1;
          }

          calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
          backproj &= mask;
          trackBox = CamShift(backproj, trackWindow,
                              TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
          
          if( trackWindow.area() <= 1 )
          {
              // Notify that the destination has been lost
              state.data = 2;
              state_pub.publish(state);

              ROS_INFO("*********DESTINATION LOST in CAMSHIFT************");
              ROS_INFO("track height %d width %d", trackWindow.height, trackWindow.width);
              
              trackObject = 0; //Disable tracking to avoid termination of node due to negative heights TBD
              int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
              trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                 trackWindow.x + r, trackWindow.y + r) &
                            Rect(0, 0, cols, rows);
          }
      }
  }
  else if( trackObject < 0 )
  {
    //If a new destination has been selected stop pausing
    paused = false;
  }

  // Find the area of the destination and publish it
  trackArea(trackWindow);

  // Find the angle of the destination wrt to the robot and publish that
  calcAngle(trackBox.center);
}

void Vision::trackArea(Rect window)
{
  sensor_msgs::RegionOfInterest roi_msg;
  roi_msg.x_offset = window.x;
  roi_msg.y_offset = window.y;
  roi_msg.height = window.height;
  roi_msg.width = window.width;
  roi_pub.publish(roi_msg);
}

void Vision::calcAngle(Point2f destCentre)
{
  std_msgs::Float32 normAngle;
  //If we have started tracking the object
  if(trackObject != 0)
  {
    normAngle.data = (destCentre.x - ((float)imgWidth/2))/((float)imgWidth/2);
    robot_angle_pub.publish(normAngle);
  }
}

Vision::~Vision()
{
}

