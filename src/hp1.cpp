#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <phoenix.h>
#include <hp1.h>

// const  DEG2RAD = 0.01745;

using namespace std;

int main(int argc, char** argv)
{
  // initialize ROS	
  ros::init(argc, argv, "hp1");

  // initialize robot	
  Hp1 *hp1 = new Hp1;

  // Main loop
  ros::Rate loop_rate(30);

  hp1->setup();
  
  while (ros::ok())
  {
    hp1->update();

    hp1->process();

    hp1->publish();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

Hp1::Hp1()
{ 
  // Register input subscriber
  input_sub = n.subscribe<hp1::input>("hp1/input", 30, &Hp1::inputCallback, this);

  // Register Leg Publishers
  lf_coxa_pub = n.advertise<std_msgs::Float64>("lf_coxa_controller/command", 1);
  lf_femur_pub = n.advertise<std_msgs::Float64>("lf_femur_controller/command", 1);
  lf_tibia_pub = n.advertise<std_msgs::Float64>("lf_tibia_controller/command", 1);

  lm_coxa_pub = n.advertise<std_msgs::Float64>("lm_coxa_controller/command", 1);
  lm_femur_pub = n.advertise<std_msgs::Float64>("lm_femur_controller/command", 1);
  lm_tibia_pub = n.advertise<std_msgs::Float64>("lm_tibia_controller/command", 1);

  lr_coxa_pub = n.advertise<std_msgs::Float64>("lr_coxa_controller/command", 1);
  lr_femur_pub = n.advertise<std_msgs::Float64>("lr_femur_controller/command", 1);
  lr_tibia_pub = n.advertise<std_msgs::Float64>("lr_tibia_controller/command", 1);

  rf_coxa_pub = n.advertise<std_msgs::Float64>("rf_coxa_controller/command", 1);
  rf_femur_pub = n.advertise<std_msgs::Float64>("rf_femur_controller/command", 1);
  rf_tibia_pub = n.advertise<std_msgs::Float64>("rf_tibia_controller/command", 1);
 
  rm_coxa_pub = n.advertise<std_msgs::Float64>("rm_coxa_controller/command", 1);
  rm_femur_pub = n.advertise<std_msgs::Float64>("rm_femur_controller/command", 1);
  rm_tibia_pub = n.advertise<std_msgs::Float64>("rm_tibia_controller/command", 1);

  rr_coxa_pub = n.advertise<std_msgs::Float64>("rr_coxa_controller/command", 1);
  rr_femur_pub = n.advertise<std_msgs::Float64>("rr_femur_controller/command", 1);
  rr_tibia_pub = n.advertise<std_msgs::Float64>("rr_tibia_controller/command", 1);

  ROS_INFO_STREAM("Hp1 Initialized");
}

void Hp1::setup()
{
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_sPS2ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  ::g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.

  ::setup();

  ROS_INFO_STREAM("Hp1 Setup"); 
}

void Hp1::update()
{
    // Do IK
    ::loop();
}

void Hp1::process()
{
  // Set Leg Message Data
  #define cPwmMult      128
  #define cPwmDiv       375  
  #define cPFConst      512 // half of our 1024 range
  
  // radians = tickToRad((((angle))* cPwmMult) / cPwmDiv +cPFConst);
  lf_coxa_msg.data =  tickToRad((((CoxaAngle1[5]))  * cPwmMult) / cPwmDiv + cPFConst);
  lf_femur_msg.data = tickToRad((((FemurAngle1[5])) * cPwmMult) / cPwmDiv + cPFConst);
  lf_tibia_msg.data = tickToRad((((TibiaAngle1[5])) * cPwmMult) / cPwmDiv + cPFConst);

  lm_coxa_msg.data =  tickToRad((((CoxaAngle1[4]))  * cPwmMult) / cPwmDiv + cPFConst);
  lm_femur_msg.data = tickToRad((((FemurAngle1[4])) * cPwmMult) / cPwmDiv + cPFConst);
  lm_tibia_msg.data = tickToRad((((TibiaAngle1[4])) * cPwmMult) / cPwmDiv + cPFConst);

  lr_coxa_msg.data =  tickToRad((((CoxaAngle1[3]))  * cPwmMult) / cPwmDiv + cPFConst);
  lr_femur_msg.data = tickToRad((((FemurAngle1[3])) * cPwmMult) / cPwmDiv + cPFConst);
  lr_tibia_msg.data = tickToRad((((TibiaAngle1[3])) * cPwmMult) / cPwmDiv + cPFConst);

  rf_coxa_msg.data =  tickToRad((((-CoxaAngle1[2]))  * cPwmMult) / cPwmDiv + cPFConst);
  rf_femur_msg.data = tickToRad((((-FemurAngle1[2])) * cPwmMult) / cPwmDiv + cPFConst);
  rf_tibia_msg.data = tickToRad((((-TibiaAngle1[2])) * cPwmMult) / cPwmDiv + cPFConst);

  rm_coxa_msg.data =  tickToRad((((-CoxaAngle1[1]))  * cPwmMult) / cPwmDiv + cPFConst);
  rm_femur_msg.data = tickToRad((((-FemurAngle1[1])) * cPwmMult) / cPwmDiv + cPFConst);
  rm_tibia_msg.data = tickToRad((((-TibiaAngle1[1])) * cPwmMult) / cPwmDiv + cPFConst);

  rr_coxa_msg.data =  tickToRad((((-CoxaAngle1[0]))  * cPwmMult) / cPwmDiv + cPFConst);
  rr_femur_msg.data = tickToRad((((-FemurAngle1[0])) * cPwmMult) / cPwmDiv + cPFConst);
  rr_tibia_msg.data = tickToRad((((-TibiaAngle1[0])) * cPwmMult) / cPwmDiv + cPFConst);
}

void Hp1::publish()
{
  // Publish Leg Message Data
  lf_coxa_pub.publish(lf_coxa_msg);
  lf_femur_pub.publish(lf_femur_msg);
  lf_tibia_pub.publish(lf_tibia_msg);

  lm_coxa_pub.publish(lm_coxa_msg);
  lm_femur_pub.publish(lm_femur_msg);
  lm_tibia_pub.publish(lm_tibia_msg);

  lr_coxa_pub.publish(lr_coxa_msg);
  lr_femur_pub.publish(lr_femur_msg);
  lr_tibia_pub.publish(lr_tibia_msg);

  rf_coxa_pub.publish(rf_coxa_msg);
  rf_femur_pub.publish(rf_femur_msg);
  rf_tibia_pub.publish(rf_tibia_msg);

  rm_coxa_pub.publish(rm_coxa_msg);
  rm_femur_pub.publish(rm_femur_msg);
  rm_tibia_pub.publish(rm_tibia_msg);

  rr_coxa_pub.publish(rr_coxa_msg);
  rr_femur_pub.publish(rr_femur_msg);
  rr_tibia_pub.publish(rr_tibia_msg);
}

void Hp1::inputCallback(const hp1::input::ConstPtr& msg)
{

if(true) 
{
  bool fAdjustLegPositions = false;

  // In an analog mode so should be OK...
  g_sPS2ErrorCnt = 0;    // clear out error count...

  // OK lets try "0" button for Start. 
  if (msg->buttons[PSB_START]) 
  {
    if (::g_InControlState.fHexOn) 
    {
      turnRobotOff();
    } 
    else 
    {
      //Turn on
      ::g_InControlState.fHexOn = 1;
      fAdjustLegPositions = true;
    }
  }

  if (::g_InControlState.fHexOn) 
  {
    // [SWITCH MODES]

    //Translate mode
    if (msg->buttons[PSB_L1]) // L1 Button Test
    {
      if (ControlMode != TRANSLATEMODE )
      {
        ControlMode = TRANSLATEMODE;
      }
      else 
      {
        if (::g_InControlState.SelectedLeg==255)
        { 
          ControlMode = WALKMODE;
        }
        else
        {
          ControlMode = SINGLELEGMODE;
        }
      }
    }

    //Rotate mode
    if (msg->buttons[PSB_L2]) // L2 Button Test
    {
      if (ControlMode != ROTATEMODE)
      {
        ControlMode = ROTATEMODE;
      }
      else 
      {
        if (::g_InControlState.SelectedLeg == 255)
        {
          ControlMode = WALKMODE;
        }
        else
        {
          ControlMode = SINGLELEGMODE;
        }
      }
    }

    //Single leg mode fNO
    if (msg->buttons[PSB_CIRCLE]) // O - Circle Button Test
    {
      if (abs(::g_InControlState.TravelLength.x)<cTravelDeadZone 
            && abs(::g_InControlState.TravelLength.z)<cTravelDeadZone 
            && abs(::g_InControlState.TravelLength.y*2)<cTravelDeadZone )   
      {
        if (ControlMode != SINGLELEGMODE) 
        {
          ControlMode = SINGLELEGMODE;
          if (::g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
          {
            ::g_InControlState.SelectedLeg=cRF; //Startleg
          }
        } 
        else 
        {
          ControlMode = WALKMODE;
          ::g_InControlState.SelectedLeg=255;
        }
      }
    }      

    //[Common functions]
    //Switch Balance mode on/off 
    if (msg->buttons[PSB_SQUARE]) // Square Button Test
    { 
      ::g_InControlState.BalanceMode = !::g_InControlState.BalanceMode;
    }

    //Stand up, sit down  
    if (msg->buttons[PSB_TRIANGLE]) // Triangle - Button Test
    {
      if (g_BodyYOffset>0)
      {
        g_BodyYOffset = 0;
      }
      else
      {
        g_BodyYOffset = 35;
      }
      fAdjustLegPositions = true;
    }

    if (msg->buttons[PSB_PAD_UP]) // D-Up - Button Test
    {
      g_BodyYOffset += 10;
      // And see if the legs should adjust...
      fAdjustLegPositions = true;
      if (g_BodyYOffset > MAX_BODY_Y)
      {
        g_BodyYOffset = MAX_BODY_Y;
      }
    }

    if (msg->buttons[PSB_PAD_DOWN] && g_BodyYOffset) // D-Down - Button Test
    {
      if (g_BodyYOffset > 10)
      {
        g_BodyYOffset -= 10;
      }
      else
      {
        g_BodyYOffset = 0;      // constrain don't go less than zero.
      }
      // And see if the legs should adjust...
      fAdjustLegPositions = true;
    }

    if (msg->buttons[PSB_PAD_RIGHT])  // D-Right - Button Test
    {
      if (::g_InControlState.SpeedControl>0) 
      {
        ::g_InControlState.SpeedControl = ::g_InControlState.SpeedControl - 50;
      }
    }

    if (msg->buttons[PSB_PAD_LEFT]) // D-Left - Button Test
    {
      if (::g_InControlState.SpeedControl<2000) 
      {
        ::g_InControlState.SpeedControl = ::g_InControlState.SpeedControl + 50;
      }
    }

    //[Walk functions]
    if (ControlMode == WALKMODE) 
    {
      //Switch gates
      if (msg->buttons[PSB_SELECT]            // Select Button Test
        && abs(::g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
        && abs(::g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(::g_InControlState.TravelLength.y*2)<cTravelDeadZone) 
      {
        ::g_InControlState.GaitType = ::g_InControlState.GaitType+1; // Go to the next gait...
        if (::g_InControlState.GaitType>=NUM_GAITS) // Make sure we did not exceed number of gaits...
        {
          ::g_InControlState.GaitType = 0;
        }
        ::GaitSelect();
      }

      //Double leg lift height
      if (msg->buttons[PSB_R1]) // R1 Button Test
      { 
        DoubleHeightOn = !DoubleHeightOn;
        if (DoubleHeightOn)
        {
          ::g_InControlState.LegLiftHeight = 80;
        }
        else
        {
          ::g_InControlState.LegLiftHeight = 50;
        }
      }

      //Double Travel Length
      if (msg->buttons[PSB_R2]) // R2 Button Test
      {
        DoubleTravelOn = !DoubleTravelOn;
      }

      // Switch between Walk method 1 && Walk method 2
      if (msg->buttons[PSB_R3]) // R3 Button Test
      {
        WalkMethod = !WalkMethod;
      }

      //Walking
      if (WalkMethod)  //(Walk Methode) 
      {  
        ::g_InControlState.TravelLength.z = (msg->axes[PSS_RY]); //Right Stick Up/Down  
      }
      else 
      {
        ::g_InControlState.TravelLength.x = -(msg->axes[PSS_LX]);
        ::g_InControlState.TravelLength.z = (msg->axes[PSS_LY]);
      }

      if (!DoubleTravelOn) //(Double travel length)
      {  
        ::g_InControlState.TravelLength.x = ::g_InControlState.TravelLength.x/2;
        ::g_InControlState.TravelLength.z = ::g_InControlState.TravelLength.z/2;
      }

      ::g_InControlState.TravelLength.y = -(msg->axes[PSS_RX])/4; //Right Stick Left/Right 
    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) 
    {
      ::g_InControlState.BodyPos.x = (msg->axes[PSS_LX])/2;
      ::g_InControlState.BodyPos.z = -(msg->axes[PSS_LY])/3;
      ::g_InControlState.BodyRot1.y = (msg->axes[PSS_RX])*2;
      g_BodyYShift = (-(msg->axes[PSS_RY])/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) 
    {
      ::g_InControlState.BodyRot1.x = (msg->axes[PSS_LY]);
      ::g_InControlState.BodyRot1.y = (msg->axes[PSS_RX])*2;
      ::g_InControlState.BodyRot1.z = (msg->axes[PSS_LX]);
      g_BodyYShift = (-(msg->axes[PSS_RY])/2);
    }

    //[Single leg functions]
    if (ControlMode == SINGLELEGMODE) 
    {
      //Switch leg for single leg control
      if (msg->buttons[PSB_SELECT]) // Select Button Test
      { 
        if (::g_InControlState.SelectedLeg<5)
        {
          ::g_InControlState.SelectedLeg = ::g_InControlState.SelectedLeg+1;
        }
        else
        {
          ::g_InControlState.SelectedLeg=0;
        }
      }

      ::g_InControlState.SLLeg.x = (msg->axes[PSS_LX])/2; //Left Stick Right/Left
      ::g_InControlState.SLLeg.y = (msg->axes[PSS_RY])/10; //Right Stick Up/Down
      ::g_InControlState.SLLeg.z = (msg->axes[PSS_LY])/2; //Left Stick Up/Down

      // Hold single leg in place
      if (msg->buttons[PSB_R2]) // R2 Button Test
      { 
        ::g_InControlState.fSLHold = !::g_InControlState.fSLHold;
      }
    }

    //Calculate walking time delay
   // ::g_InControlState.InputTimeDelay =  max(max(abs(msg->axes[PSS_LX]), abs(msg->axes[PSS_LY])), abs(msg->axes[PSS_RX]));
  }

    //Calculate g_InControlState.BodyPos.y
    ::g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    
    if (fAdjustLegPositions)
    {
    //  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
    }
  }
}

//==============================================================================
// PS2TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void Hp1::turnRobotOff(void)
{
  //Turn off
  ::g_InControlState.BodyPos.x = 0;
  ::g_InControlState.BodyPos.y = 0;
  ::g_InControlState.BodyPos.z = 0;
  ::g_InControlState.BodyRot1.x = 0;
  ::g_InControlState.BodyRot1.y = 0;
  ::g_InControlState.BodyRot1.z = 0;
  ::g_InControlState.TravelLength.x = 0;
  ::g_InControlState.TravelLength.z = 0;
  ::g_InControlState.TravelLength.y = 0;
  // g_BodyYOffset = 0;
  // g_BodyYShift = 0;
  ::g_InControlState.SelectedLeg = 255;
  ::g_InControlState.fHexOn = 0;
  // AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}

/* Convert radians to servo position offset. */
int Hp1::radToServo( float rads)
{
   float val = (rads*100)/51 * 100;
  return (int) val;
}

/* Convert servo position offset to radians*/
double Hp1::tickToRad(int tick)
{
  return ((double)tick-512) * 0.0051;
}

Hp1::~Hp1()
{

}
