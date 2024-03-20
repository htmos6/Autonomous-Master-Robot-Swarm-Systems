#include "Motor_Driver.hpp"

MotorController::MotorController(uint pin1, uint pin2, uint Ai1, uint Ai2, uint Bi1, uint Bi2, uint stb)
    :m_pin1(pin1), m_pin2(pin2), Ain1(Ai1), Ain2(Ai2), Bin1(Bi1), Bin2(Bi2), stby(stb){
    gpio_init(Ain1);
    gpio_init(Ain2);
    gpio_init(Bin1);
    gpio_init(Bin2);
    gpio_init(stby);

    gpio_set_dir(Ain1, GPIO_OUT);
    gpio_set_dir(Ain2, GPIO_OUT);
    gpio_set_dir(Bin1, GPIO_OUT);
    gpio_set_dir(Bin2, GPIO_OUT);
    gpio_set_dir(stby, GPIO_OUT);

    gpio_put(Ain1, 0);
    gpio_put(Ain2, 0);
    gpio_put(Bin1, 0);
    gpio_put(Bin2, 0);
    gpio_put(stby, 1);

    // Set up PWM channels for each motor
    pwm_set_wrap(m_pwm1, 10000);  // PWM period is 100 cycles
    pwm_set_wrap(m_pwm2, 10000);
    m_pin1 = pin1;
    m_pin2 = pin2;
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_set_clkdiv(m_pwm1, div);
    pwm_set_clkdiv(m_pwm2, div);
    pwm_set_chan_level(m_pwm1, m_channel1, 0);  // Start with 0% duty cycle
    pwm_set_chan_level(m_pwm2, m_channel2, 0);
    pwm_set_enabled(m_pwm1, true);  // Enable PWM output
    pwm_set_enabled(m_pwm2, true);
    gpio_set_function(m_pin1, GPIO_FUNC_PWM);
    gpio_set_function(m_pin2, GPIO_FUNC_PWM);
}

void MotorController::setSpeeds(float speed1, float speed2) {
    // Set the duty cycle for each motor based on the desired speed (0-1)
    
    if(speed1 > 0){
        gpio_put(Ain1, 1);
        gpio_put(Ain2, 0);
        dir1 = 1;
    }else if(speed1 < 0){
        speed1 = -1.0f * speed1;
        gpio_put(Ain1, 0);
        gpio_put(Ain2, 1);
        dir1 = -1;
    }else{
        gpio_put(Ain1, 0);
        gpio_put(Ain2, 0);
        dir1 = 1;
    }

    if(speed2 > 0){
        gpio_put(Bin1, 1);
        gpio_put(Bin2, 0);
        dir2 = 1;
    }else if(speed2 < 0){
        speed2 = -1.0f * speed2;
        gpio_put(Bin1, 0);
        gpio_put(Bin2, 1);
        dir2 = -1;
    }else{
        gpio_put(Bin1, 0);
        gpio_put(Bin2, 0);
        dir2 = 1;
    }

    uint duty1 = (uint)(speed1 * 10000);
    uint duty2 = (uint)(speed2 * 10000);
    pwm_set_chan_level(m_pwm1, m_channel1, duty1);
    pwm_set_chan_level(m_pwm2, m_channel2, duty2);
}

