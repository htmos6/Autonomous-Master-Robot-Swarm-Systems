#include "Encoder.hpp"

Encoder::Encoder(uint pin, int ticks_p_revol_in, float radius_in){
    counter = 0;
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, false, true);
    gpio_set_function(pin, GPIO_FUNC_NULL);
    ticks_p_revol = ticks_p_revol_in;
    radius = radius_in;
    encoder_map.insert({pin, this});
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true, &Encoder::encoder_cb);
}

// Define the count function
void Encoder::count(){
    counter++;
    //std::cout << counter << std::endl;
}

// Define the reset function
void Encoder::reset(){
    counter = 0;
}

void Encoder::encoder_cb(uint gpio, uint32_t events){
    if (gpio_get(gpio) && ((time_us_64() - encoder_map[gpio]->debounce) > 10000) && gpio_get(gpio)){
        encoder_map[gpio]->count();
        encoder_map[gpio]->debounce = time_us_64();
    }
}
