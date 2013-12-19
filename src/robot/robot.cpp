/* 
 * File:   robot.cpp
 * Author: viki
 * 
 * Created on April 7, 2013, 1:12 PM
 */

#include <robot/robot.h>
#include <robot/robot_states.h>

CRobot::CRobot() :
  m_pPreviousState(NULL),
  m_pCurrentState(NULL),
  fatigue(TirednessThreshold)
{
  ChangeState(Resting::Instance());
}

CRobot::~CRobot()
{
}

//-----------------------------------------------------------------------------

void CRobot::ChangeState(State* pNewState)
{
  //make sure both states are both valid before attempting to 
  //call their methods
  assert(pNewState);

  if (m_pCurrentState != NULL && m_pCurrentState != pNewState)
  {
    assert(m_pCurrentState);
    
    m_pPreviousState = m_pCurrentState;

    //call the exit method of the existing state
    m_pCurrentState->Exit(this);
  }

  //change state to the new state
  m_pCurrentState = pNewState;

  //call the entry method of the new state
  m_pCurrentState->Enter(this);
}

//-----------------------------------------------------------------------------

void CRobot::Update()
{
  if (m_pCurrentState)
  {
    m_pCurrentState->Execute(this);
  }
}

bool CRobot::isFatigued()
{
  return fatigue > TirednessThreshold;
}

void CRobot::DecreaseFatigue()
{
  fatigue--;
  if (fatigue < 0) fatigue = 0;
}

void CRobot::IncreaseFatigue()
{
  fatigue++;
}

bool CRobot::isMoving()
{
  return moving > 0;
}

void CRobot::setMoving(int value)
{
  moving = value;
}
