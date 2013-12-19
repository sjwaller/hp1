//==============================================================================
// GLOBALS - The main global definitions for the CPhenix program - still needs
//      to be cleaned up.
// This program assumes that the main files were compiled as C files
//==============================================================================
#ifndef _HEX_GLOBALS_H_
#define _HEX_GLOBALS_H_

#include <hex_cfg.h>

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP   1

#define c1DEC       10
#define c2DEC       100
#define c4DEC       10000
#define c6DEC       1000000

#define cRR         0
#define cRM         1
#define cRF         2
#define cLR         3
#define cLM         4
#define cLF         5

#define WTIMERTICSPERMSMUL      64  // BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV      125 // 
#define USEINT_TIMERAV


#define NUM_GAITS    6
#define SmDiv    4  //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, char CtrlDivider);

extern long           CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
extern long           FemurAngle1[6];   //Actual Angle of the vertical hip, decimals = 1
extern long           TibiaAngle1[6];   //Actual Angle of the knee, decimals = 1

typedef struct _Coord3D {
    long      x;
    long      y;
    long      z;
} COORD3D;

//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
     bool      fHexOn;   //Switch to turn on Phoenix
     bool      fPrev_HexOn;  //Previous loop state 
//Body position
     COORD3D        BodyPos;
     COORD3D        BodyRotOffset;      // Body rotation offset;

//Body Inverse Kinematics
     COORD3D        BodyRot1;           // X -Pitch, Y-Rotation, Z-Roll

//[gait]
     int     GaitType;   //Gait type

     short      LegLiftHeight;  //Current Travel height
     COORD3D        TravelLength;       // X-Z or Length, Y is rotation.

//[Single Leg Control]
     int     SelectedLeg;
     COORD3D        SLLeg;              // 
     bool      fSLHold;    //Single leg control mode


//[Balance]
     bool        BalanceMode;

//[TIMING]
     int     InputTimeDelay; //Delay that depends on the input to get the "sneaking" effect
     ushort     SpeedControl; //Adjustible Delay
     int           ForceGaitStepCnt;          // new to allow us to force a step even when not moving
} INCONTROLSTATE;

//-----------------------------------------------------------------------------
// Define global class objects
//-----------------------------------------------------------------------------
extern INCONTROLSTATE   g_InControlState;        // State information that controller changes

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern bool          g_fEnableServos;      // Hack to allow me to turn servo processing off...

#endif


