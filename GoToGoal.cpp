#include "GoToGoal.hpp"

double GoToGoal::step(double x_g, double y_g, double x, double y, double theta, double dt){
    dt = dt / 1000000000.0;

    // Distance between goal and robot in x direction
    double u_x = x_g - x;

    // Distance between goal and robot in y direction
    double u_y = y_g - y;

    if ((fabs(u_x) < 0.05) && (fabs(u_y) < 0.05)) {
        printf("GOoooOOOOOlLLL %f %f\\n", x, y);
        return 0;
    }

    // Angle from robot to goal
    double theta_g = atan2(u_y, u_x);

    // Error between the goal angle and robot's angle
    double e_k = theta_g - theta;
    e_k = atan2(sin(e_k), cos(e_k));

    // Error for the proportional term
    double e_P = e_k;

    // Error for the integral term.
    double e_I = E_i + e_k * dt;

    // Error for the derivative term.
    double e_D = (e_k - E_d) / dt;

    double w = Kp * e_P + Ki * e_I + Kd * e_D;
    printf("Distance from goal %f %f %f %f %f %f %f %f %f\\n", u_x, u_y, theta_g, e_k, e_P, e_I, e_D, w, dt);

    // Save errors for next time step
    E_i = e_I;
    E_d = e_k;

    return w;
}    