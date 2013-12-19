#ifndef HP1_H
#define	HP1_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <hp1/input.h>

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

    // Register Leg Publishers
    ros::Publisher rr_coxa_pub, rr_tibia_pub, rr_femur_pub;
    ros::Publisher lr_coxa_pub, lr_tibia_pub, lr_femur_pub;
    ros::Publisher rm_coxa_pub, rm_tibia_pub, rm_femur_pub;
    ros::Publisher lm_coxa_pub, lm_tibia_pub, lm_femur_pub;
    ros::Publisher rf_coxa_pub, rf_tibia_pub, rf_femur_pub;
    ros::Publisher lf_coxa_pub, lf_tibia_pub, lf_femur_pub;

    // Register Leg Messages
    std_msgs::Float64 rr_coxa_msg, rr_tibia_msg, rr_femur_msg;
    std_msgs::Float64 lr_coxa_msg, lr_tibia_msg, lr_femur_msg;
    std_msgs::Float64 rm_coxa_msg, rm_tibia_msg, rm_femur_msg;
    std_msgs::Float64 lm_coxa_msg, lm_tibia_msg, lm_femur_msg;
    std_msgs::Float64 rf_coxa_msg, rf_tibia_msg, rf_femur_msg;
    std_msgs::Float64 lf_coxa_msg, lf_tibia_msg, lf_femur_msg;

  	void inputCallback(const hp1::input::ConstPtr& msg);

public:

    Hp1();

    virtual ~Hp1();

    void setup();

    void update();

    void process();

    void publish();

	void turnRobotOff(void);
	
    int radToServo(float rads);

	double tickToRad(int tick);

};

#endif	/* HP1_H */

