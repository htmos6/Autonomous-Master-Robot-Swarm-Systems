#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"

#include "TCP_Server.hpp"
#include "Motor_Driver.hpp"
#include "Encoder.hpp"
#include "Odometry.hpp"
#include "StateControl.hpp"
#include "Pure_Pursuit.hpp"
#include "IMU.hpp"
#include <iomanip>
#include "IMU_calib.hpp"

#define WIFI_SSID "simp_wifi"
#define WIFI_PASSWORD "1den9akadar"


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// GPIO defines
// Example uses GPIO 2
#define GPIO 2


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}



std::vector<std::string> split (const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}

std::vector<double> operator-(std::vector<double> op1, std::vector<double> op2){
    return {op1[0] - op2[0], op1[1] - op2[1]};
}



int main()
{
    stdio_init_all();

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    

    // GPIO initialisation.
    // We will make this GPIO an input, and pull it up by default
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_IN);
    gpio_pull_up(GPIO);
    

    /*
    // Example of using the HW divider. The pico_divider library provides a more user friendly set of APIs 
    // over the divider (and support for 64 bit divides), and of course by default regular C language integer
    // divisions are redirected thru that library, meaning you can just use C level `/` and `%` operators and
    // gain the benefits of the fast hardware divider.
    int32_t dividend = 123456;
    int32_t divisor = -321;
    // This is the recommended signed fast divider for general use.
    divmod_result_t result = hw_divider_divmod_s32(dividend, divisor);
    printf("%d/%d = %d remainder %d\n", dividend, divisor, to_quotient_s32(result), to_remainder_s32(result));
    // This is the recommended unsigned fast divider for general use.
    int32_t udividend = 123456;
    int32_t udivisor = 321;
    divmod_result_t uresult = hw_divider_divmod_u32(udividend, udivisor);
    printf("%d/%d = %d remainder %d\n", udividend, udivisor, to_quotient_u32(uresult), to_remainder_u32(uresult));

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);



    puts("Hello, world!"); */

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }
    //run_tcp_server_test();

    /*struct tcp_pcb* listener = tcp_new();
    tcp_bind(listener, &ip_addr_any, 4242);
    listener = tcp_listen(listener);

    tcp_accept(listener, tcp_server_accept_callback);

    while (1) {
        tcp_accepted(listener);
        sleep_ms(1000);
    }*/

    MotorController motors(15, 9, 14, 13, 10, 11, 12);
    //motors.setSpeeds(0.3, 0.3);
    float wheel_radius = 0.0335;
    float wheel_base = 0.145;
    Encoder left_wheel_encoder(18, 20, wheel_radius);
    Encoder right_wheel_encoder(28, 20, wheel_radius);

    MPU6050 imu(0x68);
    sleep_ms(1000);
    //calib(&imu);
    //return 0;

    /*double ax, ay, az, gr, gp, gy;
    imu.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
    std::cout << "Gyroscope R,P,Y: " << std::setprecision(16) << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
   
    imu.reset();
    imu.startImu();
    double ang_x, ang_y, ang_z;
    int ct = 0;
    while (true){
        std::tuple<double, double, double> pose;
        pose = imu.getPosition();
        std::cout << "[POS] x: " << std::get<0>(pose) << " y: " << std::get<1>(pose) << " z: " << std::get<2>(pose) << "counter: " << ct << std::endl;
        ct++;
        imu.getAngle(0, &ang_x);
        imu.getAngle(1, &ang_y);
        imu.getAngle(2, &ang_z);
        
        std::cout << "[Ang] x: " << ang_x << " y: " << ang_y << " z: " << ang_z << std::endl;
        sleep_ms(250);
    }
    */
    
    
    
    /*float ax, ay, az, gr, gp, gy;
    std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	imu.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
    */

    //SpeedEstimator speed_meas(&left_wheel_encoder, &right_wheel_encoder, wheel_radius, wheel_base);

    /*double ang_z_sum = 0.0;
    for (int i = 0; i < 20000; i++){
        double ang_x, ang_y, ang_z;
        imu.getGyro(&ang_x, &ang_y, &ang_z);

        ang_z_sum+=ang_z;
        sleep_ms(1);
    }

    ang_z_sum/=20000.0;
    std::cout << "Z_calib: " << std::setprecision(16) << ang_z_sum << std::endl;*/

    /*double ang_x, ang_y, ang_z;
    while(true){
        imu.getAngle(0, &ang_x);
        imu.getAngle(1, &ang_y);
        imu.getAngle(2, &ang_z);

        std::cout << "Ang x: " << ang_x << " y: " << ang_y << " z: " << ang_z << std::endl;
        sleep_ms(250);
    }*/

    TCP_Server server(4242);
restart_tcp:
    server.start();

    while (true){
        std::string msg_str;
        /*int wait_for_empty = 0;
        while (server.pcb_send->snd_queuelen > 0) {
            sleep_ms(500);
            if(wait_for_empty < 10){
                wait_for_empty++;
            }else{
                wait_for_empty = 0;
                server.stop();
                goto restart_tcp;
            }
        }*/

        TCP_Server::add_cb([&](std::string msg){
            msg_str = msg;
        });

        const std::string path_begin = "Path Begin";
        while (msg_str.find(path_begin) != 0){
            std::cout << "Waiting for path..." << std::endl;
            if(!TCP_Server::is_connected){
                goto restart_tcp;
            }
            sleep_ms(1000);
        }

        std::cout << msg_str << std::endl;
        std::vector<std::string> path_x_y = split(msg_str, '\n');

        motors.setSpeeds(0, 0);

        Odometry odo(&left_wheel_encoder, &right_wheel_encoder, wheel_base);
        StateControl stControl;

        //double path[][2] = {{0,0}, {1,0}, {2,2}, {4,3}};

        std::vector<std::vector<double>> path;

        std::vector<std::string> point_x_y = split(path_x_y[1], ',');
        //path.push_back({ static_cast<double>(stoi(point_x_y[0]))/100.0, static_cast<double>(stoi(point_x_y[1]))/100.0});
        
        std::vector<double> first_point = { static_cast<double>(stoi(point_x_y[0]))/100.0, static_cast<double>(stoi(point_x_y[1]))/100.0};
        //path[0] = {0, 0};

        // Prevent drawing circle at the beginning

        for (int i = 1; i < path_x_y.size(); i++){
            std::vector<std::string> point_x_y = split(path_x_y[i], ',');
            path.push_back({ static_cast<double>(stoi(point_x_y[0]))/100.0 - first_point[0], static_cast<double>(stoi(point_x_y[1]))/100.0 - first_point[1]});
            std::cout << path[i - 1][0] << " " << path[i - 1][1] << std::endl;
        }

        std::vector<double> last2 = (path[path.size() - 1] - path[path.size() - 2]);
        double slope = -1.0*last2[0]/last2[1];
        double end_x = slope;
        double end_y = -1.0;
        double end_a = path[path.size() - 1][1] - slope*path[path.size() - 1][0];
        double end_dir = 0.0;

        if ( (path[path.size() - 2][0]*end_x + path[path.size() - 2][1]*end_y + end_a) > 0 ){
            end_dir = -1.0;
        }else{
            end_dir = 1.0;
        }

        uint64_t t, start_time, dt;

        int num_points = path.size();
        // Control loop
        double x = 0, y = 0, theta = 0;
        double lookahead_distance = 0.15;
        double kP = 1.0, kD = 0.25;
        double prev_error = 0;
        int last_target = 0;
        dt = 25;

        double linear_speed = 0.4;
        double linear_speed_target = 0.6;

        //You can play with kd value for fine tuning!
        left_wheel_encoder.reset();
        right_wheel_encoder.reset();
        imu.reset();
        odo.resetPose();
        start_time = time_us_64();
        imu.startImu();
        while(true){
            //odo.step(motors.dir1, motors.dir2);
            odo.step(motors.dir1, motors.dir2);
            std::tuple<float, float, float> pose;
            pose = odo.getPose();
            x = std::get<0>(pose);
            y = std::get<1>(pose);

            double ang_z;
            imu.getAngle(2, &ang_z);
            theta = ang_z * (M_PI/(-180.0)); //std::get<2>(pose);

            char location_echo[20];
            sprintf(location_echo, "LOC,%d,%d;", static_cast<int>((x + first_point[0])*100), static_cast<int>((y + first_point[1])*100));
            server.send(location_echo);

            double steering_angle = pure_pursuit(x, y, theta, lookahead_distance, path, num_points, last_target);
            // Calculate the error and derivative
            double error = steering_angle;
            double derivative = (error - prev_error) / static_cast<double>(dt);
            prev_error = error;
            // Calculate the control output
            double output = ((kP*error + kD*derivative)/M_PI)*0.4;
            // Convert the control output to wheel speeds
            double left_speed = linear_speed - output;
            double right_speed = linear_speed + output;
            /*double left_speed = (abs(output) <= 0.20) ? (linear_speed - output) : (0.38 - output*(1.0-0.38)/0.4);
            double right_speed = (abs(output) <= 0.20) ? (linear_speed + output) : (0.38 + output*(1.0-0.38)/0.4);
            
            if(left_speed < 0.2 && left_speed >= 0){
                left_speed = 0.2;
            }else if(left_speed > -0.2 && left_speed < 0){
                left_speed = -0.2;
            }

            if(right_speed < 0.2 && right_speed >= 0){
                right_speed = 0.2;
            }else if(right_speed > -0.2 && right_speed < 0){
                right_speed = -0.2;
            }*/

            /*double left_speed = (linear_speed - output);
            double right_speed = (linear_speed + output);*/

            std::cout << "L: " << left_speed << " R: " << right_speed << std::endl;
            /*std::pair<float, float> rpm = speed_meas.wheelSpeed(dt, 1, 1);
            std::cout << "EN_L: " << rpm.first << " rpm  EN_R: " << rpm.second << std::endl;*/

            std::cout << "x: " << x << " y: " << y << " theta: " << theta << std::endl;

            motors.setSpeeds(left_speed, right_speed);

            if (linear_speed - linear_speed_target <= -0.01){
                linear_speed+=0.01;
            }else if (linear_speed - linear_speed_target >= 0.01){
                linear_speed-=0.01;
            }

            if ( (last_target >= num_points - 4) && ( (sqrt(pow(x - path[num_points-1][0], 2) + pow(y - path[num_points-1][1], 2)) <= 0.1) || 
                 ( end_dir*(x*end_x + y*end_y + end_a) > 0 ) ) ) {
                // Stop the robot or switch to a different behavior
                motors.setSpeeds(0.0, 0.0);
                break;
            }

            sleep_ms(dt);
        }
    }

    cyw43_arch_deinit();
    return 0;
}
