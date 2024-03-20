#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <utility>
#include <math.h>
#include "Encoder.hpp"

class Odometry 
{
public:
    Odometry(Encoder* el_in, Encoder* er_in, float L_in);
    std::tuple<float, float, float> step(int left_direction = 1, int right_direction = 1);
    void resetPose();
    std::tuple<float, float, float> getPose();
private:
    Encoder* el;
    Encoder* er;
    float L;
    int wl_last_counter;
    int wr_last_counter;
    float x;
    float y;
    float theta;
    float meters_per_tick_left;
    float meters_per_tick_right;
};

#endif // ODOMETRY_H