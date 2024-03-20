#include "SpeedEstimator.hpp"

SpeedEstimator::SpeedEstimator(Encoder* leftEncoder, Encoder* rightEncoder, float radius, float distance) {
    el = leftEncoder;
    er = rightEncoder;
    R = radius;
    L = distance;
    wl_last_counter = 0;
    wr_last_counter = 0;

    // Speed converter for rpm
    wl_speed_rpm = (60.0f / static_cast<float>(el->ticks_p_revol)) * 1e9f;
    wr_speed_rpm = (60.0f / static_cast<float>(er->ticks_p_revol)) * 1e9f;
}

// Calculate wheel speeds. Call this function between 50ms to 200ms.
std::pair<float, float> SpeedEstimator::wheelSpeed(int dt, int left_direction, int right_direction) {
    int wl_delta_ticks = el->counter - wl_last_counter;
    int wr_delta_ticks = er->counter - wr_last_counter;

    wl_last_counter = el->counter;
    wr_last_counter = er->counter;

    float left_wheel_speed = static_cast<float>(left_direction * wl_delta_ticks / dt) * wl_speed_rpm;
    float right_wheel_speed = static_cast<float>(right_direction * wr_delta_ticks / dt) * wr_speed_rpm;

    return { left_wheel_speed, right_wheel_speed };
}

// Calculate robot speed
std::pair<float, float> SpeedEstimator::robotSpeed(float left_rpm, float right_rpm) {
    float left_rad_s = M_PI / 30.0f * left_rpm;
    float right_rad_s = M_PI / 30.0f * right_rpm;
    float v = R / 2.0 * (right_rad_s + left_rad_s);
    float w = R / L * (right_rad_s - left_rad_s);

    return { v, w };
}

std::pair<float, float> uni_to_diff(float v, float w, Encoder* el, Encoder* er, float L) {
    float vel_r = (2.0f * v + w * L) / static_cast<float>(2 * er->ticks_p_revol);
    float vel_l = (2.0f * v - w * L) / static_cast<float>(2 * el->ticks_p_revol);

    return { vel_l, vel_r };
}
