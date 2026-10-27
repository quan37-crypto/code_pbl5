#ifndef DHT11_H
#define DHT11_H

#include "driver/gpio.h"

typedef struct {
    float temperature;
    float humidity;
} dht11_reading;

// Hàm đọc DHT11
dht11_reading dht11_read(gpio_num_t pin);

#endif