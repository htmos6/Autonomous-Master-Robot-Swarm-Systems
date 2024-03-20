#include "StateControl.hpp"

StateControl::StateControl(){
    go_to_goal = new GoToGoalSt();
    StateMap.insert(std::make_pair(INIT_ST, new InitSt()));
    StateMap.insert(std::make_pair(GO_TO_GOAL_ST, go_to_goal));
    StateMap.insert(std::make_pair(AT_THE_GOAL_ST, new AtTheGoalSt()));
    StateMap.insert(std::make_pair(NEXT_GOAL_ST, go_to_goal));
}

void StateControl::Step(){
    uint8_t next_state = StateMap[CurrentState]->run(inputs, outputs);

    if (next_state != CurrentState){
        StateMap[CurrentState]->exit(inputs, outputs);
        CurrentState = next_state;
        StateMap[CurrentState]->entry(inputs, outputs);
    }
}

uint8_t InitSt::run(Inputs& input, Outputs& output) {
    return GO_TO_GOAL_ST;
}

void GoToGoalSt::entry(Inputs& input, Outputs& output){
    // Set goal and next goal (in the next entry)
    goal = array_of_goals[next_goal];
    next_goal = (next_goal + 1) % array_of_goals.size();
    std::cout << "GoToGoal state " << goal[0] << ", " << goal[1] << ", " << next_goal << std::endl;

    // Initiate rate limit variable (assume that the robot is always stopped when
    // entering autonomous mode)
    leftPrevCmd = 0;
    rightPrevCmd = 0;

    init_sgn_x = (input.x - goal[0] >= 0) ? 1 : -1;
    init_sgn_y = (input.y - goal[1] >= 0) ? 1 : -1;

    // Create an instance of the PID controller
    controller = new GoToGoal();
}

double GoToGoalSt::limit(double value, double downLimit, double upLimit){
    return value >= upLimit ? upLimit : (value <= downLimit ? downLimit : value);
}

double GoToGoalSt::rateLimit(double value, double ctrlVar, double upLimit, double downLimit){
    ctrlVar = limit(value, (downLimit + ctrlVar), (upLimit + ctrlVar));
    return ctrlVar;
}

uint8_t GoToGoalSt::run(Inputs& input, Outputs& output){
    uint8_t next_state = GO_TO_GOAL_ST;

    // Run controller
    double w = controller->step(goal[0], goal[1], input.x, input.y, input.theta, input.dt);

    // Estimate the motor outputs with fixed speed of 0.05
    double left, right;
    std::pair<float, float> diff = uni_to_diff(0.03, w, input.el, input.er, input.L);
    left = diff.first;                      // 0.03
    right = diff.second;

    std::cout << std::endl << "Motor commands from PID: left, right" << left << right << std::endl;

    double left2, right2;
    if (left > right){
        left2 = left/left;
        right2 = right/left;
    }else{
        left2 = left/right;
        right2 = right/right;
    }

    left2 = left;
    right2 = right;

    std::cout << "left 2, right 2" << left2 << right2 << std::endl;

    double left3 = rateLimit(left2, leftPrevCmd, 0.9, -0.9);
    double right3 = rateLimit(right2, rightPrevCmd, 0.9, -0.9);
    left3 = left3 >=0.5 ? left3 : 0.5;
    right3 = right3 >= 0.5 ? right3 : 0.5;

    std::cout << "left3, right 3" << left3 << right3 << std::endl;


/*
    left = diff.first;                      // 0.03
    right = diff.second;

    std::cout << std::endl << "Motor commands from PID: left, right" << left << right << std::endl;

    double left2, right2;
    //if (left > right){
    //    left2 = left/left;
    //    right2 = right/left;
    //}else{
    //    left2 = left/right;
    //    right2 = right/right;
    //}
    left2 = left;
    right2 = right;


    std::cout << "left 2, right 2" << left2 << right2 << std::endl;

    double left3 = rateLimit(left2, leftPrevCmd, 1, -1);
    double right3 = rateLimit(right2, rightPrevCmd, 1, -1);
    left3 = (left3 >=0.5 && left3 <=-0.5) ? left3 : (left3 >= 0 ? 0.5 : -0.5);
    right3 = (right3 >=0.5 && right3 <=-0.5) ? right3 : (right3 >= 0 ? 0.5 : -0.5);
*/


    left2 = left2 >= 0 ? left2 : 0;
    right2 = right2 >= 0 ? right2 : 0;
    output.left_motor = left3;
    output.right_motor = right3;
    std::cout << "GoToGoal outputs:" << output.left_motor << output.right_motor << std::endl;
    
    if ( (abs(input.x - goal[0]) < 0.15) && (abs(input.y - goal[1]) < 0.15)){
        if (next_goal != 0){
            next_state = NEXT_GOAL_ST;
        }else{
            next_state = AT_THE_GOAL_ST;
        }
    }else if( (((input.x - goal[0] >= 0) ? 1 : -1) != init_sgn_x) ||
              (((input.y - goal[1] >= 0) ? 1 : -1) != init_sgn_y) ){
        if (next_goal != 0){
            next_state = NEXT_GOAL_ST;
        }else{
            next_state = AT_THE_GOAL_ST;
        }
    }

    return next_state;
}


void AtTheGoalSt::entry(Inputs& input, Outputs& output){
    output.left_motor = 0;
    output.right_motor = 0;
}

uint8_t AtTheGoalSt::run(Inputs& input, Outputs& output){
    uint8_t next_state = AT_THE_GOAL_ST;

    output.left_motor = 0;
    output.right_motor = 0;

    return AT_THE_GOAL_ST;
}
