//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
// This version is converted to run on HP1 robot
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//   Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
// ============================================================================
// This version of the Phoenix code was ported over to C++ by sjwaller@gmail.com
//
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include <phoenix.h>

bool g_fEnableServos = true;

//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//Min / imax values
int cCoxaMin1[] = {
  (int)cRRCoxaMin1,  (int)cRMCoxaMin1,  (int)cRFCoxaMin1,  (int)cLRCoxaMin1,  (int)cLMCoxaMin1,  (int)cLFCoxaMin1};
int cCoxaMax1[] = {
  (int)cRRCoxaMax1,  (int)cRMCoxaMax1,  (int)cRFCoxaMax1,  (int)cLRCoxaMax1,  (int)cLMCoxaMax1,  (int)cLFCoxaMax1};
int cFemurMin1[] ={
  (int)cRRFemurMin1, (int)cRMFemurMin1, (int)cRFFemurMin1, (int)cLRFemurMin1, (int)cLMFemurMin1, (int)cLFFemurMin1};
int cFemurMax1[] ={
  (int)cRRFemurMax1, (int)cRMFemurMax1, (int)cRFFemurMax1, (int)cLRFemurMax1, (int)cLMFemurMax1, (int)cLFFemurMax1};
int cTibiaMin1[] ={
  (int)cRRTibiaMin1, (int)cRMTibiaMin1, (int)cRFTibiaMin1, (int)cLRTibiaMin1, (int)cLMTibiaMin1, (int)cLFTibiaMin1};
int cTibiaMax1[] = {
  (int)cRRTibiaMax1, (int)cRMTibiaMax1, (int)cRFTibiaMax1, (int)cLRTibiaMax1, (int)cLMTibiaMax1, (int)cLFTibiaMax1};

//Leg Lengths
int cCoxaLength[] = {
  (int)cRRCoxaLength,  (int)cRMCoxaLength,  (int)cRFCoxaLength,  (int)cLRCoxaLength,  (int)cLMCoxaLength,  (int)cLFCoxaLength};
int cFemurLength[] = {
  (int)cRRFemurLength, (int)cRMFemurLength, (int)cRFFemurLength, (int)cLRFemurLength, (int)cLMFemurLength, (int)cLFFemurLength};
int cTibiaLength[] = {
  (int)cRRTibiaLength, (int)cRMTibiaLength, (int)cRFTibiaLength, (int)cLRTibiaLength, (int)cLMTibiaLength, (int)cLFTibiaLength};

//Body Offsets [distance between the center of the body and the center of the coxa]
int cOffsetX[] = {
  (int)cRROffsetX, (int)cRMOffsetX, (int)cRFOffsetX, (int)cLROffsetX, (int)cLMOffsetX, (int)cLFOffsetX};
int cOffsetZ[] = {
  (int)cRROffsetZ, (int)cRMOffsetZ, (int)cRFOffsetZ, (int)cLROffsetZ, (int)cLMOffsetZ, (int)cLFOffsetZ};

//Default leg angle
int cCoxaAngle1[] = {
  (int)cRRCoxaAngle1, (int)cRMCoxaAngle1, (int)cRFCoxaAngle1, (int)cLRCoxaAngle1, (int)cLMCoxaAngle1, (int)cLFCoxaAngle1};

//Start positions for the leg
int cInitPosX[] = {
  (int)cRRInitPosX, (int)cRMInitPosX, (int)cRFInitPosX, (int)cLRInitPosX, (int)cLMInitPosX, (int)cLFInitPosX};
int cInitPosY[] = {
  (int)cRRInitPosY, (int)cRMInitPosY, (int)cRFInitPosY, (int)cLRInitPosY, (int)cLMInitPosY, (int)cLFInitPosY};
int cInitPosZ[] = {
  (int)cRRInitPosZ, (int)cRMInitPosZ, (int)cRFInitPosZ, (int)cLRInitPosZ, (int)cLMInitPosZ, (int)cLFInitPosZ};


//====================================================================
//[ANGLES]
long           CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
long           FemurAngle1[6];   //Actual Angle of the vertical hip, decimals = 1
long           TibiaAngle1[6];   //Actual Angle of the knee, decimals = 1

//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

long           LegPosX[6];    //Actual X Posion of the Leg
long           LegPosY[6];    //Actual Y Posion of the Leg
long           LegPosZ[6];    //Actual Z Posion of the Leg

//--------------------------------------------------------------------
//[VARIABLES]
int            Index;                    //Index universal used
int            LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
short           AngleDeg1;        //Input Angle in degrees, decimals = 1
short           sin4;             //Output Sinus of the given Angle, decimals = 4
short           cos4;            //Output Cosinus of the given Angle, decimals = 4
short           AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
short           AtanX;            //Input X
short           AtanY;            //Input Y
short           Atan4;            //ArcTan2 output
long            XYhyp2;            //Output presenting Hypotenuse of X and Y

//Body Inverse Kinematics
double           PosX;            //Input position of the feet X
double           PosZ;            //Input position of the feet Z
double           PosY;            //Input position of the feet Y
long            BodyFKPosX;        //Output Position X of feet with Rotation
long            BodyFKPosY;        //Output Position Y of feet with Rotation
long            BodyFKPosZ;        //Output Position Z of feet with Rotation


//Leg Inverse Kinematics
long            IKFeetPosX;        //Input position of the Feet X
long            IKFeetPosY;        //Input position of the Feet Y
long            IKFeetPosZ;        //Input Position of the Feet Z
bool         IKSolution;        //Output true if the solution is possible
bool         IKSolutionWarning;    //Output true if the solution is NEARLY possible
bool         IKSolutionError;    //Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;    //Start time of the calculation cycles
unsigned long   lTimerEnd;        //End time of the calculation cycles
int            CycleTime;        //Total Cycle time

short            ServoMoveTime;        //Time for servo updates
short            PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOABAL]
//--------------------------------------------------------------------

// Define our global Input Control State object
INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

//--bool         g_InControlState.fHexOn;            //Switch to turn on Phoenix
//--bool         g_InControlState.fPrev_HexOn;        //Previous loop state 
//--------------------------------------------------------------------
//[Balance]
long            TotalTransX;

long            TotalTransZ;
long            TotalTransY;
long            TotalYBal1;
long            TotalXBal1;
long            TotalZBal1;

//[Single Leg Control]
int            PrevSelectedLeg;
bool         AllDown;

//[gait]

short   NomGaitSpeed;   //Nominal speed of the gait
short           TLDivFactor;         //Number of steps that a leg is on the floor while walking
short           NrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
int            LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4
int            FrontDownPos;        //Where the leg should be put down to ground

bool         HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height    

bool         TravelRequest;        //Temp to check if the gait is in motion
int            StepsInGait;         //Number of steps in gait

bool         LastLeg;             //TRUE when the current leg is the last leg of the sequence
int            GaitStep;            //Actual Gait step

int            GaitLegNr[6];        //Init position of the leg

int            GaitLegNrIn;         //Input Number of the leg

long            GaitPosX[6];         //Array containing Relative X position corresponding to the Gait
long            GaitPosY[6];         //Array containing Relative Y position corresponding to the Gait
long            GaitPosZ[6];         //Array containing Relative Z position corresponding to the Gait
long            GaitRotY[6];         //Array containing Relative Y rotation corresponding to the Gait

bool         fWalking;            //  True if the robot are walking
int            bExtraCycle;         // Forcing some extra timed cycles for avoiding "end of gait bug"

//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup()
{
  //Checks to see if our Servo Driver support a GP Player
  //    DBGSerial.write("Program Start\n\r");
  // debug stuff
  usleep(1000);

  //Tars Init Positions
  for (LegIndex= 0; LegIndex <= 5; LegIndex++ )
  {
    LegPosX[LegIndex] = (long)(cInitPosX[LegIndex]);    //Set start positions for each leg
    LegPosY[LegIndex] = (long)(cInitPosY[LegIndex]);
    LegPosZ[LegIndex] = (long)(cInitPosZ[LegIndex]);  
  }

  //Single leg control. Make sure no leg is selected
  g_InControlState.SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;

  //Body Positions
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;

  //Body Rotations
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.BodyRotOffset.x = 0;
  g_InControlState.BodyRotOffset.y = 0;        //Input Y offset value to adjust centerpoint of rotation
  g_InControlState.BodyRotOffset.z = 0;


  //Gait
  g_InControlState.GaitType = 1;  // 0; Devon wanted 
  g_InControlState.BalanceMode = 0;
  g_InControlState.LegLiftHeight = 50;
  g_InControlState.ForceGaitStepCnt = 0;    // added to try to adjust starting positions depending on height...
  GaitStep = 1;
  GaitSelect();

  // Servo Driver
  ServoMoveTime = 150;
  g_InControlState.fHexOn = 1;
}


//=============================================================================
// Loop: the main Loop function
//=============================================================================
long millis()
{
   sys_time_t t;
   system_time(&t);
   return time_to_msec(t);
}

void loop()
{
  //Start time
  lTimerStart = millis(); 
  
  //Single leg control
  SingleLegControl ();

  //Gait
  GaitSeq();

  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;

  if (g_InControlState.BalanceMode) 
  {
    for (LegIndex = 0; LegIndex <= 2; LegIndex++)   // balance calculations for all Right legs
    {
      BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex], 
      LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
      (LegPosY[LegIndex]-cInitPosY[LegIndex])+GaitPosY[LegIndex], LegIndex);
    }

    for (LegIndex = 3; LegIndex <= 5; LegIndex++)    // balance calculations for all Right legs
    { 
      BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex], 
      LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
      (LegPosY[LegIndex]-cInitPosY[LegIndex])+GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }

  //Reset IKsolution indicators 
  IKSolution = 0 ;
  IKSolutionWarning = 0; 
  IKSolutionError = 0 ;

  //Do IK for all Right legs
  for (LegIndex = 0; LegIndex <=2; LegIndex++) 
  {    
    BodyFK(-LegPosX[LegIndex]+g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);

    LegIK (LegPosX[LegIndex]-g_InControlState.BodyPos.x+BodyFKPosX-(GaitPosX[LegIndex] - TotalTransX), 
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs  
  for (LegIndex = 3; LegIndex <=5; LegIndex++) 
  {
    BodyFK(LegPosX[LegIndex]-g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);

    LegIK (LegPosX[LegIndex]+g_InControlState.BodyPos.x-BodyFKPosX+GaitPosX[LegIndex] - TotalTransX,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }
  
  //Check mechanical limits
  CheckAngles();

  //Drive Servos
  if (g_InControlState.fHexOn) 
  {
    //Calculate Servo Move time
    if ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) ||
      (abs(g_InControlState.TravelLength.y*2)>cTravelDeadZone)) 
    {         
      ServoMoveTime = NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;

      //Add aditional delay when Balance mode is on
      if (g_InControlState.BalanceMode)
      {
        ServoMoveTime = ServoMoveTime + 100;
      }
    } 
    else //Movement speed excl. Walking
    {
      ServoMoveTime = 200 + g_InControlState.SpeedControl;
    }

    // Finding any incident of GaitPos/Rot <>0:
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) 
    {
      if ( (GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit)
        || (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit) 
        || (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit))    
      {

        bExtraCycle = NrLiftedPos + 1;//For making sure that we are using timed move until all legs are down
        break;
      }
    }

    if (bExtraCycle>0)
    { 
      long lTimeWaitEnd;
      bExtraCycle--;
      fWalking = !(bExtraCycle==0);

      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

      do {
        // Wait the appropriate time, call any background process while waiting...
      } 
      while (millis() < lTimeWaitEnd);
    }

    // Only do commit if we are actually doing something...
 //   g_ServoDriver.CommitServoDriver(ServoMoveTime);

  } 
  else 
  {
    //Turn the bot off - May need to add ajust here...
    if (g_InControlState.fPrev_HexOn || (AllDown= 0)) 
    {
      ServoMoveTime = 600;
 //     g_ServoDriver.CommitServoDriver(ServoMoveTime);
      usleep(60000);
    } 
    else 
    {
 //     g_ServoDriver.FreeServos();
 //     Eyes = 0;
    }

    usleep(2000);  // give a pause between times we call if nothing is happening
  }

  PrevServoMoveTime = ServoMoveTime;

  //Store previous g_InControlState.fHexOn State
  if (g_InControlState.fHexOn)
    g_InControlState.fPrev_HexOn = 1;
  else
    g_InControlState.fPrev_HexOn = 0;
}

//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{
  //Check if all legs are down
  AllDown = (LegPosY[cRF]==cInitPosY[cRF]) && 
    (LegPosY[cRM]==cInitPosY[cRM]) && 
    (LegPosY[cRR]==cInitPosY[cRR]) && 
    (LegPosY[cLR]==cInitPosY[cLR]) && 
    (LegPosY[cLM]==cInitPosY[cLM]) && 
    (LegPosY[cLF]==cInitPosY[cLF]);

  if (g_InControlState.SelectedLeg<=5) {
    if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[g_InControlState.SelectedLeg] = cInitPosY[g_InControlState.SelectedLeg]-20;

        //Store current status
        PrevSelectedLeg = g_InControlState.SelectedLeg;
      } 
      else {//Return prev leg back to the init position
        LegPosX[PrevSelectedLeg] = cInitPosX[PrevSelectedLeg];
        LegPosY[PrevSelectedLeg] = cInitPosY[PrevSelectedLeg];
        LegPosZ[PrevSelectedLeg] = cInitPosZ[PrevSelectedLeg];
      }
    } 
    else if (!g_InControlState.fSLHold) {
      //LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
      LegPosY[g_InControlState.SelectedLeg] = cInitPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;// Using DIY remote Zenta prefer it this way
      LegPosX[g_InControlState.SelectedLeg] = cInitPosX[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.x;
      LegPosZ[g_InControlState.SelectedLeg] = cInitPosZ[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.z;     
    }
  } 
  else {//All legs to init position
    if (!AllDown) {
      for(LegIndex = 0; LegIndex <= 5;LegIndex++) {
        LegPosX[LegIndex] = cInitPosX[LegIndex];
        LegPosY[LegIndex] = cInitPosY[LegIndex];
        LegPosZ[LegIndex] = cInitPosZ[LegIndex];
      }
    } 
    if (PrevSelectedLeg!=255)
      PrevSelectedLeg = 255;
  }
}

#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 60
#define DEFAULT_SLOW_GAIT 70
#endif
//--------------------------------------------------------------------
void GaitSelect(void)
{
  //Gait selector
  switch (g_InControlState.GaitType)  {
  case 0:
    //Ripple Gait 12 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 7;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 11;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;      
    StepsInGait = 12;    
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 1:
    //Tripod 8 steps
    GaitLegNr[cLR] = 5;
    GaitLegNr[cRF] = 1;
    GaitLegNr[cLM] = 1;
    GaitLegNr[cRR] = 1;
    GaitLegNr[cLF] = 5;
    GaitLegNr[cRM] = 5;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 4;
    StepsInGait = 8; 
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 2:
    //Triple Tripod 12 step
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 4;
    GaitLegNr[cRR] = 5;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 10;
    GaitLegNr[cLR] = 11;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;
    StepsInGait = 12; 
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 3:
    // Triple Tripod 16 steps, use 5 lifted positions
    GaitLegNr[cRF] = 4;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 6;
    GaitLegNr[cLF] = 12;
    GaitLegNr[cRM] = 13;
    GaitLegNr[cLR] = 14;

    NrLiftedPos = 5;
    FrontDownPos = 3;
    LiftDivFactor = 4;
    HalfLiftHeigth = 1;
    TLDivFactor = 10;
    StepsInGait = 16; 
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 4:
    //Wave 24 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 21;
    GaitLegNr[cLM] = 5;

    GaitLegNr[cRR] = 13;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 17;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 20;      
    StepsInGait = 24;        
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 5:
    //Tripod 6 steps
    GaitLegNr[cLR] = 4;
    GaitLegNr[cRF] = 1;
    GaitLegNr[cLM] = 1;

    GaitLegNr[cRR] = 1;
    GaitLegNr[cLF] = 4;
    GaitLegNr[cRM] = 4;

    NrLiftedPos = 2;
    FrontDownPos = 1;
    LiftDivFactor = 2;
    HalfLiftHeigth = 1;
    TLDivFactor = 4;      
    StepsInGait = 6;        
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  }
}    

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
  //Check if the Gait is in motion
  TravelRequest = (abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) 
    || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone) || (g_InControlState.ForceGaitStepCnt != 0) || fWalking;

  //Calculate Gait sequence
  LastLeg = 0;
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) { // for all legs
    if (LegIndex == 5) // last leg
      LastLeg = 1 ;

    Gait(LegIndex);
  }    // next leg

  // If we have a force count decrement it now... 
  if (g_InControlState.ForceGaitStepCnt)
    g_InControlState.ForceGaitStepCnt--;
}


//--------------------------------------------------------------------
//[GAIT]
void Gait (int GaitCurrentLegNr)
{
  //Clear values under the cTravelDeadZone
  if (!TravelRequest) {    
    g_InControlState.TravelLength.x=0;
    g_InControlState.TravelLength.z=0;
    g_InControlState.TravelLength.y=0;//Gait NOT in motion, return to home position
  }
  //Leg middle up position OK
  //Gait in motion                                                                                    

  if ((TravelRequest && (NrLiftedPos==1 || NrLiftedPos==3 || NrLiftedPos==5) && 
    GaitStep==GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep==GaitLegNr[GaitCurrentLegNr] && ((abs(GaitPosX[GaitCurrentLegNr])>2) || 
    (abs(GaitPosZ[GaitCurrentLegNr])>2) || (abs(GaitRotY[GaitCurrentLegNr])>2)))) { //Up
    GaitPosX[GaitCurrentLegNr] = 0;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
    GaitPosZ[GaitCurrentLegNr] = 0;
    GaitRotY[GaitCurrentLegNr] = 0;

  }
  //Optional Half heigth Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos==2 && GaitStep==GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos>=3 && 
    (GaitStep==GaitLegNr[GaitCurrentLegNr]-1 || GaitStep==GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1))))
    && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/LiftDivFactor;

  }    
  // _A_    
  // Optional Half heigth front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+1 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/LiftDivFactor;

  }

  //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos==5 && (GaitStep==GaitLegNr[GaitCurrentLegNr]-2 ))) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/2;

  }     

  //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos==5) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+2 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-2)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;

  }
  //_B_
  //Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
  //For example, where is the case of GaitStep==GaitLegNr[GaitCurrentLegNr]+2 executed when NRLiftedPos=3?
  else if ((GaitStep==GaitLegNr[GaitCurrentLegNr]+FrontDownPos || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-FrontDownPos))
    && GaitPosY[GaitCurrentLegNr]<0) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;       
    GaitPosY[GaitCurrentLegNr] = 0; 

  }

  //Move body forward      
  else {
    GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (g_InControlState.TravelLength.x/TLDivFactor);
    GaitPosY[GaitCurrentLegNr] = 0; 
    GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (g_InControlState.TravelLength.z/TLDivFactor);
    GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (g_InControlState.TravelLength.y/TLDivFactor);

  }


  //Advance to the next step
  if (LastLeg)  {  //The last leg in this step
    GaitStep++;
    if (GaitStep>StepsInGait)
      GaitStep = 1;
  }
}  


//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg (long PosX, long PosZ, long PosY, int BalLegNr)
{
  long            CPR_X;            //Final X value for centerpoint of rotation
  long            CPR_Y;            //Final Y value for centerpoint of rotation
  long            CPR_Z;            //Final Z value for centerpoint of rotation
  long            lAtan;

  //Calculating totals from center of the body to the feet
  CPR_Z = cOffsetZ[BalLegNr] + PosZ;
  CPR_X = cOffsetX[BalLegNr] + PosX;
  CPR_Y = 150 + PosY;        // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

  TotalTransY += (long)PosY;
  TotalTransZ += (long)CPR_Z;
  TotalTransX += (long)CPR_X;

  lAtan = GetATan2(CPR_X, CPR_Z);
  TotalYBal1 += (lAtan*1800) / 31415;

  lAtan = GetATan2 (CPR_X, CPR_Y);
  TotalZBal1 += ((lAtan*1800) / 31415) -900; //Rotate balance circle 90 deg

  lAtan = GetATan2 (CPR_Z, CPR_Y);
  TotalXBal1 += ((lAtan*1800) / 31415) - 900; //Rotate balance circle 90 deg
}

//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{
  TotalTransZ = TotalTransZ/BalanceDivFactor;
  TotalTransX = TotalTransX/BalanceDivFactor;
  TotalTransY = TotalTransY/BalanceDivFactor;

  if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
    TotalYBal1 -=  1800;
  else
    TotalYBal1 += 1800;

  if (TotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
    TotalZBal1 += 3600;

  if (TotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
    TotalXBal1 += 3600;

  //Balance rotation
  TotalYBal1 = -TotalYBal1/BalanceDivFactor;
  TotalXBal1 = -TotalXBal1/BalanceDivFactor;
  TotalZBal1 =  TotalZBal1/BalanceDivFactor;
}


//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1)
{
  short        ABSAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1
  //Get the absolute value of AngleDeg
  if (AngleDeg1 < 0)
    ABSAngleDeg1 = AngleDeg1 *-1;
  else
    ABSAngleDeg1 = AngleDeg1;

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg1 < 0)    //Negative values
    AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
  else                //Positive values
  AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));

  if (AngleDeg1>=0 && AngleDeg1<=900)     // 0 to 90 deg
  {
    sin4 = (GetSin[AngleDeg1/5]);             // 5 is the presision (0.5) of the table
    cos4 = (GetSin[(900-(AngleDeg1))/5]);
  }     

  else if (AngleDeg1>900 && AngleDeg1<=1800)     // 90 to 180 deg
  {
    sin4 = (GetSin[(900-(AngleDeg1-900))/5]); // 5 is the presision (0.5) of the table    
    cos4 = -(GetSin[(AngleDeg1-900)/5]);            
  }    
  else if (AngleDeg1>1800 && AngleDeg1<=2700) // 180 to 270 deg
  {
    sin4 = -(GetSin[(AngleDeg1-1800)/5]);     // 5 is the presision (0.5) of the table
    cos4 = -(GetSin[(2700-AngleDeg1)/5]);
  }    

  else if(AngleDeg1>2700 && AngleDeg1<=3600) // 270 to 360 deg
  {
    sin4 = -(GetSin[(3600-AngleDeg1)/5]); // 5 is the presision (0.5) of the table    
    cos4 = (GetSin[(AngleDeg1-2700)/5]);            
  }
}    


//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
long GetArcCos(short cos4)
{
  bool NegativeValue;
  
  //Check for negative value
  if (cos4<0)
  {
    cos4 = -cos4;
    NegativeValue = 1;
  }
  else
    NegativeValue = 0;

  //Limit cos4 to his maximal value
  cos4 = std::min(cos4,(short)c4DEC);

  if ((cos4>=0) && (cos4<9000))
  {
    AngleRad4 = (int)GetACos[cos4/79];
    AngleRad4 = ((long)AngleRad4*616)/c1DEC;            //616=acos resolution (pi/2/255) ;
  }    
  else if ((cos4>=9000) && (cos4<9900))
  {
    AngleRad4 = (int)GetACos[(cos4-9000)/8+114];
    AngleRad4 = (long)((long)AngleRad4*616)/c1DEC;             //616=acos resolution (pi/2/255) 
  }
  else if ((cos4>=9900) && (cos4<=10000))
  {
    AngleRad4 = (int)GetACos[(cos4-9900)/2+227];
    AngleRad4 = (long)((long)AngleRad4*616)/c1DEC;             //616=acos resolution (pi/2/255) 
  }

  //Add negative sign
  if (NegativeValue)
    AngleRad4 = 31416 - AngleRad4;

  return AngleRad4;
}    

unsigned long isqrt32 (unsigned long n) //
{
  unsigned long root;
  unsigned long remainder;
  unsigned long  place;

  root = 0;
  remainder = n;
  place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

  while (place > remainder)
    place = place >> 2;
  while (place)
  {
    if (remainder >= root + place)
    {
      remainder = remainder - root - place;
      root = root + (place << 1);
    }
    root = root >> 1;
    place = place >> 2;
  }
  return root;
}


//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY)
{
  XYhyp2 = isqrt32(((long)AtanX*AtanX*c4DEC) + ((long)AtanY*AtanY*c4DEC));
  GetArcCos (((long)AtanX*(long)c6DEC) /(long) XYhyp2);

  if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));  
    Atan4 = -AngleRad4;
  else
    Atan4 = AngleRad4;
  return Atan4;
}    

//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS) 
//BodyRotX        - Global Input pitch of the body 
//BodyRotY        - Global Input rotation of the body 
//BodyRotZ        - Global Input roll of the body 
//RotationY       - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB            - Sin buffer for BodyRotX
//CosB            - Cos buffer for BodyRotX
//SinG            - Sin buffer for BodyRotZ
//CosG            - Cos buffer for BodyRotZ
//BodyFKPosX      - Output Position X of feet with Rotation 
//BodyFKPosY      - Output Position Y of feet with Rotation 
//BodyFKPosZ      - Output Position Z of feet with Rotation
void BodyFK (short PosX, short PosZ, short PosY, short RotationY, int BodyIKLeg) 
{
  short            SinA4;          //Sin buffer for BodyRotX calculations
  short            CosA4;          //Cos buffer for BodyRotX calculations
  short            SinB4;          //Sin buffer for BodyRotX calculations
  short            CosB4;          //Cos buffer for BodyRotX calculations
  short            SinG4;          //Sin buffer for BodyRotZ calculations
  short            CosG4;          //Cos buffer for BodyRotZ calculations
  short            CPR_X;            //Final X value for centerpoint of rotation
  short            CPR_Y;            //Final Y value for centerpoint of rotation
  short            CPR_Z;            //Final Z value for centerpoint of rotation

  //Calculating totals from center of the body to the feet 
  CPR_X = cOffsetX[BodyIKLeg] + PosX + g_InControlState.BodyRotOffset.x;
  CPR_Y = PosY + g_InControlState.BodyRotOffset.y;         //Define centerpoint for rotation along the Y-axis
  CPR_Z = cOffsetZ[BodyIKLeg] + PosZ + g_InControlState.BodyRotOffset.z;

  //Successive global rotation matrix: 
  //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 

  //First calculate sinus and cosinus for each rotation: 
  GetSinCos (g_InControlState.BodyRot1.x+TotalXBal1);
  SinG4 = sin4;
  CosG4 = cos4;

  GetSinCos (g_InControlState.BodyRot1.z+TotalZBal1); 
  SinB4 = sin4;
  CosB4 = cos4;

  GetSinCos (g_InControlState.BodyRot1.y+(RotationY*c1DEC)+TotalYBal1) ;

  SinA4 = sin4;
  CosA4 = cos4;

  //Calcualtion of rotation matrix: 
  BodyFKPosX = ((long)CPR_X*c2DEC - ((long)CPR_X*c2DEC*CosA4/c4DEC*CosB4/c4DEC - (long)CPR_Z*c2DEC*CosB4/c4DEC*SinA4/c4DEC 
    + (long)CPR_Y*c2DEC*SinB4/c4DEC ))/c2DEC;
  BodyFKPosZ = ((long)CPR_Z*c2DEC - ( (long)CPR_X*c2DEC*CosG4/c4DEC*SinA4/c4DEC + (long)CPR_X*c2DEC*CosA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
    + (long)CPR_Z*c2DEC*CosA4/c4DEC*CosG4/c4DEC - (long)CPR_Z*c2DEC*SinA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
    - (long)CPR_Y*c2DEC*CosB4/c4DEC*SinG4/c4DEC ))/c2DEC;
  BodyFKPosY = ((long)CPR_Y  *c2DEC - ( (long)CPR_X*c2DEC*SinA4/c4DEC*SinG4/c4DEC - (long)CPR_X*c2DEC*CosA4/c4DEC*CosG4/c4DEC*SinB4/c4DEC 
    + (long)CPR_Z*c2DEC*CosA4/c4DEC*SinG4/c4DEC + (long)CPR_Z*c2DEC*CosG4/c4DEC*SinA4/c4DEC*SinB4/c4DEC 
    + (long)CPR_Y*c2DEC*CosB4/c4DEC*CosG4/c4DEC ))/c2DEC;
}  

//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, int LegIKLegNr)
{
  unsigned long    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
  unsigned long    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
  unsigned long    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
  short            IKFeetPosXZ;    //Diagonal direction from Input X and Z
  long            Temp1;            
  long            Temp2;            
  long            T3;

  //Calculate IKCoxaAngle and IKFeetPosXZ
  GetATan2 (IKFeetPosX, IKFeetPosZ);
  CoxaAngle1[LegIKLegNr] = (((long)Atan4*180) / 3141) + (short)(*(&cCoxaAngle1[LegIKLegNr]));

  //Length between the Coxa and tars [foot]
  IKFeetPosXZ = XYhyp2/c2DEC;

  //Using GetAtan2 for solving IKA1 and IKSW
  //IKA14 - Angle between SW line and the ground in radians
  IKA14 = GetATan2 (IKFeetPosY, IKFeetPosXZ-(int)(*(&cCoxaLength[LegIKLegNr])));
 
  //IKSW2 - Length between femur axis and tars
  IKSW2 = XYhyp2;

  //IKA2 - Angle of the line S>W with respect to the femur in radians
  Temp1 = ((((long)cFemurLength[LegIKLegNr]*cFemurLength[LegIKLegNr]) - ((long)cTibiaLength[LegIKLegNr]*cTibiaLength[LegIKLegNr]))*c4DEC + ((long)IKSW2*IKSW2));
  Temp2 = (long)(2*cFemurLength[LegIKLegNr])*c2DEC * (unsigned long)IKSW2;
  T3 = Temp1 / (Temp2/c4DEC);
  IKA24 = GetArcCos (T3 );
  
  //IKFemurAngle
  FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900; // + CFEMURHORNOFFSET1(LegIKLegNr);//Normal

  //IKTibiaAngle
  Temp1 = ((((long)cFemurLength[LegIKLegNr]*cFemurLength[LegIKLegNr]) + ((long)cTibiaLength[LegIKLegNr]*cTibiaLength[LegIKLegNr]))*c4DEC - ((long)IKSW2*IKSW2));
  Temp2 = (2*cFemurLength[LegIKLegNr]*cTibiaLength[LegIKLegNr]);
  GetArcCos (Temp1 / Temp2);
  TibiaAngle1[LegIKLegNr] = -(900-(long)AngleRad4*180/3141);

  //Set the Solution quality    
  if((short)IKSW2 < ((short)(cFemurLength[LegIKLegNr]+cTibiaLength[LegIKLegNr]-30)*c2DEC))
    IKSolution = 1;
  else
  {
    if((short)IKSW2 < ((short)(cFemurLength[LegIKLegNr]+cTibiaLength[LegIKLegNr])*c2DEC)) 
      IKSolutionWarning = 1;
    else
      IKSolutionError = 1;
  }
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{
  for (LegIndex = 0; LegIndex <=5; LegIndex++)
  {
    CoxaAngle1[LegIndex]  = std::min(std::max(CoxaAngle1[LegIndex], (long)cCoxaMin1[LegIndex]), (long)cCoxaMax1[LegIndex]);
    FemurAngle1[LegIndex] = std::min(std::max(FemurAngle1[LegIndex], (long)cFemurMin1[LegIndex]), (long)cFemurMax1[LegIndex]);
    TibiaAngle1[LegIndex] = std::min(std::max(TibiaAngle1[LegIndex], (long)cTibiaMin1[LegIndex]), (long)cTibiaMax1[LegIndex]);
  }
}

//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body 
//            rotation and translation much smoother 
//--------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, int CtrlDivider)
{

  if (CtrlMoveOut < (CtrlMoveInp - 4))
    return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
  else if (CtrlMoveOut > (CtrlMoveInp + 4))
    return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);

  return CtrlMoveInp;
}
