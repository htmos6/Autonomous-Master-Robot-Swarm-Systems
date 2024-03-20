#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H
#include "Encoder.hpp"
#include <math.h>

class SpeedEstimator 
{
private:
    Encoder *el, *er;
    float R, L;
    int wl_last_counter, wr_last_counter;
    float wl_speed_rpm, wr_speed_rpm;

public:
    // Constructor
    SpeedEstimator(Encoder* leftEncoder, Encoder* rightEncoder, float radius, float distance);
    // Calculate wheel speeds. Call this function between 50ms to 200ms.
    std::pair<float, float> wheelSpeed(int dt, int left_direction = 1, int right_direction = 1);

    // Calculate robot speed
    std::pair<float, float> robotSpeed(float left_rpm, float right_rpm);
};

std::pair<float, float> uni_to_diff(float v, float w, Encoder* el, Encoder* er, float L);

#endif // SPEED_ESTIMATOR_H