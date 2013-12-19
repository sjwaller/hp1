#ifndef ROBOT_H
#define	ROBOT_H

#include <iostream>
#include <cassert>
#include "state.h"

class State;

using namespace std;

//above this value a robot is thirsty
const int ThirstLevel = 25;

//above this value a robot is sleepy
const int TirednessThreshold = 50;

class CRobot
{
public:

    CRobot();
    virtual ~CRobot();

    State* m_pPreviousState;
    
    State* m_pCurrentState;
    
    std::string theState;    

    void Update();

    void ChangeState(State* new_state);

    bool isFatigued();
    
    void DecreaseFatigue();
    
    void IncreaseFatigue();
    
    bool isMoving();
    
    void setMoving(int value);
    
private:
    
    int fatigue;
    int moving;

};

#endif	/* ROBOT_H */

