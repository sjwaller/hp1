#ifndef ROBOT_STATES_H
#define	ROBOT_STATES_H

#include "state.h"

class CRobot;

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

class Resting : public State
{
private:

    Resting()
    {
    }

    //copy ctor and assignment should be private
    Resting(const Resting&);
    Resting& operator=(const Resting&);

public:

    //this is a singleton
    static Resting* Instance();

    virtual void Enter(CRobot* robot);

    virtual void Execute(CRobot* robot);

    virtual void Exit(CRobot* robot);
};

class Wandering : public State
{
private:

    Wandering()
    {
    }

    //copy ctor and assignment should be private
    Wandering(const Wandering&);
    Wandering& operator=(const Wandering&);

public:

    //this is a singleton
    static Wandering* Instance();

    virtual void Enter(CRobot* robot);

    virtual void Execute(CRobot* robot);

    virtual void Exit(CRobot* robot);
};


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

class Searching : public State
{
private:

    Searching()
    {
    }

    //copy ctor and assignment should be private
    Searching(const Searching&);
    Searching& operator=(const Searching&);

public:

    //this is a singleton
    static Searching* Instance();

    virtual void Enter(CRobot* robot);

    virtual void Execute(CRobot* robot);

    virtual void Exit(CRobot* robot);
};

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

class Working : public State
{
private:

    Working()
    {
    }

    //copy ctor and assignment should be private
    Working(const Working&);
    Working& operator=(const Working&);

public:

    //this is a singleton
    static Working* Instance();

    virtual void Enter(CRobot* robot);

    virtual void Execute(CRobot* robot);

    virtual void Exit(CRobot* robot);
};


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

class Travelling : public State
{
private:

    Travelling()
    {
    }

    //copy ctor and assignment should be private
    Travelling(const Travelling&);
    Travelling& operator=(const Travelling&);

public:

    //this is a singleton
    static Travelling* Instance();

    virtual void Enter(CRobot* robot);

    virtual void Execute(CRobot* robot);

    virtual void Exit(CRobot* robot);
};

#endif	/* ROBOT_STATES_H */

