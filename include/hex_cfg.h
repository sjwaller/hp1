
//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the PhantomX robot.
//    Will first define to use their commander unit.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.0
//   - First Release
//
//====================================================================
#ifndef HEX_CFG_H
#define HEX_CFG_H

//====================================================================
#define  DEFAULT_GAIT_SPEED 50  // Default gait speed  - Will depend on what Servos you are using...
#define  DEFAULT_SLOW_GAIT  50  // Had a couple different speeds...

//====================================================================

//--------------------------------------------------------------------
// Define which pins(sevo IDS go with which joint

#define cRRCoxaPin      8   //Rear Right leg Hip Horizontal
#define cRRFemurPin     10   //Rear Right leg Hip Vertical
#define cRRTibiaPin     12   //Rear Right leg Knee

#define cRMCoxaPin      14  //Middle Right leg Hip Horizontal
#define cRMFemurPin     16  //Middle Right leg Hip Vertical
#define cRMTibiaPin     18  //Middle Right leg Knee

#define cRFCoxaPin      2  //Front Right leg Hip Horizontal
#define cRFFemurPin     4  //Front Right leg Hip Vertical
#define cRFTibiaPin     6   //Front Right leg Knee

#define cLRCoxaPin      7   //Rear Left leg Hip Horizontal
#define cLRFemurPin     9   //Rear Left leg Hip Vertical
#define cLRTibiaPin     11   //Rear Left leg Knee

#define cLMCoxaPin      13   //Middle Left leg Hip Horizontal
#define cLMFemurPin     15   //Middle Left leg Hip Vertical
#define cLMTibiaPin     17  //Middle Left leg Knee

#define cLFCoxaPin      1   //Front Left leg Hip Horizontal
#define cLFFemurPin     3   //Front Left leg Hip Vertical
#define cLFTibiaPin     5   //Front Left leg Knee

//--------------------------------------------------------------------
//[MIN/MAX ANGLES] - Start off assume same as Phoenix...#define cRRCoxaMin1    -700    //Mechanical limits of the Right Rear Leg, decimals = 1
#define cXXTibiaMin1    -700
#define cXXTibiaMax1    900

#define cRRCoxaMin1    -700
#define cRRCoxaMax1    700
#define cRRFemurMin1    -900
#define cRRFemurMax1    900
#define cRRTibiaMin1    cXXTibiaMin1
#define cRRTibiaMax1    cXXTibiaMax1

#define cRMCoxaMin1    -700    //Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1     700
#define cRMFemurMin1     -900
#define cRMFemurMax1     900
#define cRMTibiaMin1    cXXTibiaMin1
#define cRMTibiaMax1     cXXTibiaMax1

#define cRFCoxaMin1    -700    //Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1     700
#define cRFFemurMin1    -900
#define cRFFemurMax1    900
#define cRFTibiaMin1    cXXTibiaMin1
#define cRFTibiaMax1    cXXTibiaMax1

#define cLRCoxaMin1    -700    //Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1     700
#define cLRFemurMin1     -900
#define cLRFemurMax1     900
#define cLRTibiaMin1    cXXTibiaMin1
#define cLRTibiaMax1     cXXTibiaMax1

#define cLMCoxaMin1    -700    //Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1     700
#define cLMFemurMin1     -900
#define cLMFemurMax1     900
#define cLMTibiaMin1    cXXTibiaMin1
#define cLMTibiaMax1     cXXTibiaMax1

#define cLFCoxaMin1     -700    //Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1     700
#define cLFFemurMin1     -900
#define cLFFemurMax1     900
#define cLFTibiaMin1    cXXTibiaMin1
#define cLFTibiaMax1     cXXTibiaMax1

//--------------------------------------------------------------------
//[Joint offsets]
// This allows us to calibrate servos to some fixed position, and then adjust them by moving theim
// one or more servo horn clicks.  This requires us to adjust the value for those servos by 15 degrees
// per click.  This is used with the T-Hex 4DOF legs
//First calibrate the servos in the 0 deg position using the SSC-32 reg offsets, then:
//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     52    // PhantomX leg dimensions.
#define cXXFemurLength    83
#define cXXTibiaLength    140

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength

//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -450   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -450   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1

#define X_COXA      60  // MM between front and back legs /2
#define Y_COXA      60  // MM between front/back legs /2
#define M_COXA      100  // MM between two middle legs /2

#define cRROffsetX      -60     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      120     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX      -100    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX      -60     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -120    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      60      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      120     //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX      100     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX      60      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -120    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 147
#define CHexInitXZCos60  104        // COS(45) = .707
#define CHexInitXZSin60  104    // sin(45) = .707
#define CHexInitY	 25 //30

// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS 2
#define MAX_BODY_Y  150

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60

#endif // HEX_CFG_H
