#ifndef ENCODER_H
#define ENCODER_H

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include <map>
#include <iostream>


class Encoder 
{
public:
    int ticks_p_revol;
    int counter;
    float radius;
    // Constructor
    Encoder(uint pin, int ticks_p_revol_in, float radius_in);
    void count();
    void reset();
    uint64_t debounce = 0;
    static void encoder_cb(uint gpio, uint32_t events);
};

static std::map<uint,Encoder*> encoder_map;

#endif // ENCODER_H