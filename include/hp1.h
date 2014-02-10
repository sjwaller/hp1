#ifndef HP1_H
#define	HP1_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hp1/input.h>
#include <hp1/robot.h>

//#include "robot/robot.h"

//[CONSTANTS]
#define PSB_SELECT        0
#define PSB_L3            1  
#define PSB_R3            2
#define PSB_START         3
#define PSB_PAD_UP        4  
#define PSB_PAD_RIGHT     5
#define PSB_PAD_DOWN      6
#define PSB_PAD_LEFT      7  
#define PSB_L2            8 
#define PSB_R2            9  
#define PSB_L1            10  
#define PSB_R1            11 
#define PSB_TRIANGLE      12
#define PSB_CIRCLE        13
#define PSB_CROSS         14
#define PSB_SQUARE        15  

#define PSS_LX            0
#define PSS_LY            1
#define PSS_RX            2
#define PSS_RY            3

#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3

#define cTravelDeadZone   4      //The deadzone for the analog input from the remote
#define MAXPS2ERRORCNT    5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y        100
#endif

// Static vars
static short       g_BodyYOffset; 
static short       g_sPS2ErrorCnt;
static short       g_BodyYShift;
static int         ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;

class Hp1 //: public CRobot
{
private:
    ros::NodeHandle n;

    ros::Subscriber input_sub;

    ros::Publisher robot_pub;

    hp1::robot robot;

  	void inputCallback(const hp1::input::ConstPtr& msg);

public:

    Hp1();

    virtual ~Hp1();

    void setup();

    void update();

    void process();

    void publish();

	void turnRobotOff(void);

};

#endif	/* HP1_H */

