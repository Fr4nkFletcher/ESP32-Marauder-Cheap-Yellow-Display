#ifndef _DHT_SENSOR_H
#define _DHT_SENSOR_H

#include "hardware/gpio.h"

typedef struct {
	float humidity;
	float temp_celsius;
} dht_reading;

void read_from_dht(dht_reading *result, uint pin);

#endif
