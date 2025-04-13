#include "dht.h"

#include "pico/time.h"
#include <stdio.h>

// Thanks to Alasdair for the example code

const uint MAX_TIMINGS = 85;
const uint LED_PIN = 16;

void read_from_dht(dht_reading *result, uint pin) {
	int data[5] = { 0, 0, 0, 0, 0 };
	uint last = 1;
	uint j = 0;

	gpio_init(pin);
	gpio_pull_up(pin);
	gpio_set_dir(pin, GPIO_OUT);
	gpio_put(pin, 0);
	sleep_ms(30);
	gpio_set_dir(pin, GPIO_IN);

	// gpio_put(LED_PIN, 1);
	for ( uint i = 0; i < MAX_TIMINGS; i++ ) {
		uint count = 0;
		while ( gpio_get(pin) == last ) {
			count++;
			sleep_us(1);
			if (count == 255) break;
		}
		last = gpio_get(pin);
		if (count == 255) break;

		if ((i >= 4) && (i % 2 == 0)) {
			data[j / 8] <<= 1;
			if (count > 16) data[j / 8] |= 1;
			j++;
		}
	}
	// gpio_put(LED_PIN, 0);

	printf("%02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4]);
	result->humidity = data[0];
	result->temp_celsius = data[2];
	if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
		// result->humidity = (float)((data[0] << 8) + data[1]) / 10;
		// if ( result->humidity > 100 ) {
		// 	result->humidity = data[0];
		// }
		// result->temp_celsius = (float)(((data[2] & 0x7F) << 8) + data[3]) / 10;
		// if (result->temp_celsius > 125) {
		// 	result->temp_celsius = data[2];
		// }
		// if (data[2] & 0x80) {
		// 	result->temp_celsius = -result->temp_celsius;
		// }
	} else  {
		printf("Bad data\n");
	}
}
