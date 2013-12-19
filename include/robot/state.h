#ifndef STATE_H
#define STATE_H

class CRobot;

class State
{
public:

    virtual ~State()
    {
    }

    //this will execute when the state is entered
    virtual void Enter(CRobot*) = 0;

    //this is the state's normal update function
    virtual void Execute(CRobot*) = 0;

    //this will execute when the state is exited. (My word, isn't
    //life full of surprises... ;o))
    virtual void Exit(CRobot*) = 0;

};

#endif