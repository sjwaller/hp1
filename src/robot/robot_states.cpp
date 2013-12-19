/* 
 * File:   robot_states.cpp
 * Author: viki
 * 
 * Created on April 7, 2013, 1:06 PM
 */

#include <robot/robot_states.h>
#include <robot/state.h>
#include <robot/robot.h>

//------------------------------------------------methods for Resting

Resting* Resting::Instance()
{
  static Resting instance;
  return &instance;
}

void Resting::Enter(CRobot* robot)
{
  robot->theState = "start rest";
}

void Resting::Execute(CRobot* robot)
{
  robot->theState = "Resting";

  if (robot->isFatigued())
  {
    robot->DecreaseFatigue();
  }
  else
  {
    // Return to previous state?
    if(robot->m_pPreviousState != NULL)
    {
      robot->ChangeState(robot->m_pPreviousState);
    }
  }
}

void Resting::Exit(CRobot* robot)
{
  robot->theState = "stop rest";
}

//------------------------------------------------methods for Wandering

Wandering* Wandering::Instance()
{
  static Wandering instance;
  return &instance;
}

void Wandering::Enter(CRobot* robot)
{
  robot->theState = "start wander";
}

void Wandering::Execute(CRobot* robot)
{
  robot->theState = "Wandering";
  robot->IncreaseFatigue();

  if (robot->isFatigued())
  {
    robot->ChangeState(Resting::Instance());
  }
}

void Wandering::Exit(CRobot* robot)
{
  robot->theState = "stop wander";
}

//------------------------------------------------methods for Searching

Searching* Searching::Instance()
{
  static Searching instance;
  return &instance;
}

void Searching::Enter(CRobot* robot)
{
  robot->theState = "start search";
}

void Searching::Execute(CRobot* robot)
{
  robot->theState = "Searching";
  robot->IncreaseFatigue();

  if (robot->isFatigued())
  {
    robot->ChangeState(Resting::Instance());
  }
}

void Searching::Exit(CRobot* robot)
{
  robot->theState = "stop search";
}


//------------------------------------------------methods for Traveling

Travelling* Travelling::Instance()
{
  static Travelling instance;
  return &instance;
}

void Travelling::Enter(CRobot* robot)
{
  robot->theState = "start travel";
}

void Travelling::Execute(CRobot* robot)
{
  robot->theState = "Traveling";
  robot->setMoving(1);
  robot->IncreaseFatigue();

  if (robot->isFatigued())
  {
    robot->ChangeState(Resting::Instance());
  }
}

void Travelling::Exit(CRobot* robot)
{
  robot->setMoving(0);
  robot->theState = "stop travel";
}

//------------------------------------------------methods for Working

Working* Working::Instance()
{
  static Working instance;
  return &instance;
}

void Working::Enter(CRobot* robot)
{
  robot->theState = "start work";
}

void Working::Execute(CRobot* robot)
{
  robot->theState = "Working";
  robot->IncreaseFatigue();

  if (robot->isFatigued())
  {
    robot->ChangeState(Resting::Instance());
  }
}

void Working::Exit(CRobot* robot)
{
  robot->theState = "stop work";
}
