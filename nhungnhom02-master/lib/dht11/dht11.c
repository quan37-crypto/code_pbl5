#include "dht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static int wait_for_state(gpio_num_t pin, int state, int timeout_us) {
    int count = 0;
    while (gpio_get_level(pin) != state) {
        if (count++ > timeout_us) return -1;
        ets_delay_us(1);
    }
    return count;
}

dht11_reading dht11_read(gpio_num_t pin) {
    dht11_reading ret = {-1.0, -1.0}; 
    uint8_t data[5] = {0};
    
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0); ets_delay_us(20000); 
    gpio_set_level(pin, 1); ets_delay_us(40);
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    if (wait_for_state(pin, 0, 80) == -1) return ret;
    if (wait_for_state(pin, 1, 80) == -1) return ret;
    if (wait_for_state(pin, 0, 80) == -1) return ret;

    for (int i = 0; i < 40; i++) {
        if (wait_for_state(pin, 1, 50) == -1) return ret;
        int len = wait_for_state(pin, 0, 70);
        if (len == -1) return ret;
        if (len > 40) data[i / 8] |= (1 << (7 - (i % 8)));
    }
    
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ret.humidity = data[0]; 
        ret.temperature = data[2];
    }
    return ret;
}