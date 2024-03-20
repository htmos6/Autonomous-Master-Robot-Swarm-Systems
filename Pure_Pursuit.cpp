#include "Pure_Pursuit.hpp"

//double path[][2] = {{0,0}, {1,1}, {2,0}, {3,1}, {4,0}, {5,1}};

// Define the Pure Pursuit algorithm
double pure_pursuit(double x, double y, double theta, double lookahead_distance, std::vector<std::vector<double>> path, int num_points, int& last_target) {
    // Find the closest point on the path
    double closest_distance = INFINITY;
    int closest_index = 0;
    for (int i = 0; i < num_points; i++) {
        double dx = path[i][0] - x;
        double dy = path[i][1] - y;
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_index = i;
        }
    }
    // Find the target point
    int target_index = closest_index;
    while ((target_index < num_points-1) && sqrt((path[target_index][0] - x)*(path[target_index][0] - x) + (path[target_index][1] - y)*(path[target_index][1] - y)) < lookahead_distance) {
        target_index++;
    }

    if (target_index < last_target){
        target_index = last_target;
    }else if (target_index - last_target > 1){
        target_index = last_target + 1;
    }

    last_target = target_index;

    std::cout << "Target x: " << path[target_index][0] << " y: " << path[target_index][1] << std::endl;
    // Calculate the steering angle
    double dx = path[target_index][0] - x;
    double dy = path[target_index][1] - y;
    double target_theta = atan2(dy, dx);
    double steering_angle = target_theta - theta;
    if (steering_angle > M_PI) steering_angle -= 2*M_PI;
    if (steering_angle < -M_PI) steering_angle += 2*M_PI;
    return steering_angle;
}

/*std::pair<double, double> pp_motor(std::tuple<float, float, float> pose){

}*/
