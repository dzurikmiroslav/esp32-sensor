#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "ble.h"
#include "sensor.h"

#define LOG_TAG "MAIN"

#define SLEEP_TIME CONFIG_SLEEP_TIME

typedef struct
{
    uint8_t length;
    uint8_t type;
    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;
    uint8_t battery;
} __attribute__((packed)) beacon_data_t;

beacon_data_t beacon_data = {
    .length = sizeof(beacon_data_t) - 1,
    .type = 0xFC};

void config_beacon_data()
{
    sensor_data_t sensor_data = sensor_read();

    beacon_data.pressure = sensor_data.pressure;
    beacon_data.temperature = sensor_data.temperature;
    beacon_data.humidity = sensor_data.humidity;
    beacon_data.battery = 100; // TODO
}

void app_main()
{
    ESP_LOGI(LOG_TAG, "Starting...");

    ESP_ERROR_CHECK(ble_init());
    ESP_ERROR_CHECK(sensor_init());

    config_beacon_data();
    ESP_LOGD(LOG_TAG, "Set beacon");
    ESP_LOG_BUFFER_HEX_LEVEL(LOG_TAG, (uint8_t *)&beacon_data, sizeof(beacon_data_t), ESP_LOG_DEBUG);
    set_beacon_data((uint8_t *)&beacon_data, sizeof(beacon_data_t));

    vTaskDelay(70 / portTICK_PERIOD_MS);

    ESP_LOGI(LOG_TAG, "Running time: %lums, gonna sleep...", clock());
    esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000);
    esp_deep_sleep_start();
}