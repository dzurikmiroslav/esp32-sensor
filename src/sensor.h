#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

typedef struct
{
   float pressure;
   float temperature;
   float humidity;
   uint8_t battery;
} __attribute__((packed)) espnow_sensor_data_t;

#endif /* SENSOR_H_ */