#ifndef BLE_H_
#define BLE_H_

#include <esp_err.h>

esp_err_t ble_init();

void set_beacon_data(uint8_t *raw_data, uint32_t raw_data_len);

#endif /* BLE_H_ */