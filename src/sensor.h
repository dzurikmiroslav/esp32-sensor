#ifndef SENSOR_H_
#define SENSOR_H_

#include <esp_system.h>

typedef struct
{
    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;
} sensor_data_t;

esp_err_t sensor_init();

sensor_data_t sensor_read();

#endif /* SENSOR_H_ */