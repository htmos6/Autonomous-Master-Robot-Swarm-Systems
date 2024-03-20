#ifndef STATE_CONTROL_H
#define STATE_CONTROL_H

#include <string>
#include <iostream>
#include <vector>
#include "pico/stdlib.h"
#include "Encoder.hpp"
#include "SpeedEstimator.hpp"
#include "GoToGoal.hpp"

#define INIT_ST 0
#define GO_TO_GOAL_ST 1
#define AT_THE_GOAL_ST 2
#define NEXT_GOAL_ST 3

struct Inputs{
    int autonomous = 0;
    double x = 0;
    double y = 0;
    double theta = 0;
    double dt = 0;
    Encoder* el;
    Encoder* er;
    double L = 0;
};

struct Outputs{
    double left_motor = 0;
    double right_motor = 0;
};

class State
{
public:
    virtual uint8_t run(Inputs& input, Outputs& output) = 0;
    virtual void entry(Inputs& input, Outputs& output) {}
    virtual void exit(Inputs& input, Outputs& output) {}
};

class InitSt : public State
{
public:
    uint8_t run(Inputs& input, Outputs& output) override;
};

class GoToGoalSt : public State
{
public:
    int next_goal = 0;
    int init_sgn_x = 0;
    int init_sgn_y = 0;
    std::vector<std::vector<double>> array_of_goals{ {1,0}, {1,1}, {0,1}, {0, 0} };
    //std::vector<std::vector<double>> array_of_goals;
    std::vector<double> goal;
    double leftPrevCmd = 0;
    double rightPrevCmd = 0;
    GoToGoal* controller;

    void entry(Inputs& input, Outputs& output) override;

    double limit(double value, double downLimit, double upLimit);

    double rateLimit(double value, double ctrlVar, double upLimit, double downLimit);

    uint8_t run(Inputs& input, Outputs& output) override;
};

class AtTheGoalSt : public State
{
public:
    void entry(Inputs& input, Outputs& output) override;

    uint8_t run(Inputs& input, Outputs& output) override;
};

class StateControl{
public:
    Inputs inputs;
    Outputs outputs;
    uint8_t CurrentState = INIT_ST;

    std::map<uint8_t, State*> StateMap;
    GoToGoalSt* go_to_goal;


    StateControl();
    void Step();


};


#endif // STATE_CONTROL_H