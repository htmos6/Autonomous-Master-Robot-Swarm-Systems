#ifndef GO_TO_GOAL_H
#define GO_TO_GOAL_H
#include "stdio.h"
#include "math.h"

class GoToGoal {
public:
    double E_d = 0;
    double E_i = 0;

    // PID gains
    double Kp = 0.1;
    //double Kp = 0.01;
    double Ki = 10;
    //double Ki = 0.0;
    double Kd = 1.0;

    double step(double x_g, double y_g, double x, double y, double theta, double dt);
};

#endif // GO_TO_GOAL_H
