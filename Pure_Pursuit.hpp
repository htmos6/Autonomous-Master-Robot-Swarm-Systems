#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
#include <cmath>
#include <utility>
#include <tuple>
#include <iostream>
#include <vector>

//double path[][2];
double pure_pursuit(double x, double y, double theta, double lookahead_distance, std::vector<std::vector<double>> path, int num_points, int& last_target);
//std::pair<double, double> pp_motor(std::tuple<float, float, float> pose);

#endif // PURE_PURSUIT_H