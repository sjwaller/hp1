#include <cmath>
#include <math.h>
#include <iostream>
#include <input.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input");

  Input input;
  
  ros::spin();
}

Input::Input() :
  scale_linear_(255),
  scale_angular_(180),
  deadzone_(10)
{
  n.param("scale_linear_", scale_linear_, scale_linear_);
  n.param("scale_angular_", scale_angular_, scale_angular_);
  n.param("deadzone_", deadzone_, deadzone_);

  joy_pub = n.advertise<hp1::input>("hp1/input", 1);
  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &Input::joyCallback, this);
}

void Input::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
      hp1::input input;

      input.buttons = joy->buttons; 
      input.axes = joy->axes;
      input.header = joy->header;
      
      // SCALE AXES
      input.axes[0] = scale_linear_ * joy->axes[0]; // LS l/r
      input.axes[1] = scale_linear_ * -joy->axes[1]; // LS u/d
      
      input.axes[2] = scale_angular_ * joy->axes[2]; // RS l/r
      input.axes[3] = scale_angular_ * joy->axes[3]; // RS u/d
      
      input.axes[4] = scale_linear_ * joy->axes[4]; // D UP
      input.axes[5] = scale_linear_ * joy->axes[5]; // D RIGHT
      input.axes[6] = scale_linear_ * joy->axes[6]; // D DOWN
      input.axes[7] = scale_linear_ * joy->axes[7]; // D LEFT
      input.axes[8] = scale_linear_ * joy->axes[8]; // L2
      input.axes[9] = scale_linear_ * joy->axes[9]; // R2
      input.axes[10] = scale_linear_ * joy->axes[10]; // L1
      input.axes[11] = scale_linear_ * joy->axes[11]; // R1
      input.axes[12] = scale_linear_ * joy->axes[12]; // TRIANGLE
      input.axes[13] = scale_linear_ * joy->axes[13]; // CIRCLE
      input.axes[14] = scale_linear_ * joy->axes[14]; // CROSS
      input.axes[15] = scale_linear_ * joy->axes[15]; // SQUARE
      
      input.axes[16] = (scale_angular_ * 0.4 * joy->axes[16]); // ACCL L/R
      input.axes[17] = -(scale_angular_ * 0.4 * joy->axes[17]); // ACCL F/B
      input.axes[18] = 0; //100 * joy->axes[18]; // ACCL
  
      for(int i = 0; i < 16; i++)
      {
        if(abs(input.axes[i]) < deadzone_)
        {
          input.axes[i] = 0;
        }
        else
        {
          input.axes[i] = input.axes[i];
        }
      }
      
      
      // BUTTONS
      // 0 - SELECT
      // 1 - L3
      // 2 - R3
      // 3 - START
      // 4 - D UP
      // 5 - D RIGHT
      // 6 - D DOWN
      // 7 - D LEFT
      // 8 - L2
      // 9 - R2
      // 10 - L1
      // 11 - R1 
      // 12 - TRIANGLE
      // 13 - CIRCLE
      // 14 - CROSS
      // 15 - SQUARE
      // 16 - PS
        
      joy_pub.publish(input);      
}
