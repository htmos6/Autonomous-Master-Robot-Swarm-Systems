#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

class MotorController {
public:
    MotorController(uint pin1, uint pin2, uint Ai1, uint Ai2, uint Bi1, uint Bi2, uint stb);

    void setSpeeds(float speed1, float speed2);

    int dir1 = 1;
    int dir2 = 1;

private:
    uint m_pin1, m_pin2;
    uint m_pwm1 = pwm_gpio_to_slice_num(m_pin1);  // Get the PWM slice for each motor
    uint m_pwm2 = pwm_gpio_to_slice_num(m_pin2);
    uint m_channel1 = pwm_gpio_to_channel(m_pin1);
    uint m_channel2 = pwm_gpio_to_channel(m_pin2);
    int freq = 1500;

    uint Ain1, Ain2, Bin1, Bin2;
    uint stby;
};

#endif // MOTOR_DRIVER_H