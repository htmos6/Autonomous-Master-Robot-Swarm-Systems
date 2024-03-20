#include "Odometry.hpp"

Odometry::Odometry(Encoder* el_in, Encoder* er_in, float L_in){
    /*
        Initialize odometry
        el - left wheel encoder
        er - right wheel encoder
        L - distance between the wheels in meters
    */
    el = el_in;
    er = er_in;
    L = L_in;
    wl_last_counter = 0;
    wr_last_counter = 0;
    x = 0;
    y = 0;
    theta = 0;
    // Calculate meters per tick
    meters_per_tick_left = (2.0f * M_PI * el->radius) / static_cast<float>(el->ticks_p_revol);
    meters_per_tick_right = (2.0f * M_PI * er->radius) / static_cast<float>(er->ticks_p_revol);
}

std::tuple<float, float, float> Odometry::step(int left_direction, int right_direction){
    /*
        Call this function periodically to update robot pose estimiation.
    */
    // Calculate the delta for ticks since last read
    int delta_ticks_left = (el->counter - wl_last_counter) * left_direction;
    int delta_ticks_right = (er->counter - wr_last_counter) * right_direction;

    // Update counters to next read
    wl_last_counter = el->counter;
    wr_last_counter = er->counter;

    // Calculate new pose
    float Dl = meters_per_tick_left * delta_ticks_left;
    float Dr = meters_per_tick_right * delta_ticks_right;
    float Dc = (Dr + Dl) / 2;

    float x_dt = Dc * cos(theta);
    float y_dt = Dc * sin(theta);
    float theta_dt = (Dr - Dl) / L;

    x = x + x_dt;
    y = y + y_dt;
    theta = theta + theta_dt;

    return {x, y, theta};
}

void Odometry::resetPose(){
    x = 0;
    y = 0;
    theta = 0;
}

std::tuple<float, float, float> Odometry::getPose(){
    return {x, y, theta};
}