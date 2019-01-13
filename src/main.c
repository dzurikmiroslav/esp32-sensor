#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event_loop.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <time.h>
#include <string.h>

#include "sdkconfig.h"
#include "bme.h"
#include "sensor.h"

#define LOG_TAG "MAIN"

#define SLEEP_TIME CONFIG_SLEEP_TIME
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

static uint8_t sensor_mac[ESP_NOW_ETH_ALEN] = CONFIG_SENSOR_MAC;
static uint8_t station_mac[ESP_NOW_ETH_ALEN] = CONFIG_STATION_MAC;

static SemaphoreHandle_t espnow_send_lock;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static void wifi_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, sensor_mac));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, 0));

#if CONFIG_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_MODE_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    xSemaphoreGive(espnow_send_lock);
}

static void espnow_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    esp_now_peer_info_t peer;
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, station_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

static espnow_sensor_data_t fetch_data()
{
    espnow_sensor_data_t ret;
    bme_data_t data = bme_read();

    ret.pressure = data.pressure;
    ret.temperature = data.temperature;
    ret.humidity = data.humidity;
    ret.battery = 100; // TODO
    return ret;
}

void app_main()
{
    ESP_LOGI(LOG_TAG, "Starting...");

    bme_init();
    wifi_init();
    espnow_init();

    espnow_send_lock = xSemaphoreCreateBinary();

    espnow_sensor_data_t espnow_data = fetch_data();
    uint8_t *bs = (uint8_t *)malloc(sizeof(espnow_sensor_data_t));
    memcpy(bs, &espnow_data, sizeof(espnow_sensor_data_t));
    esp_now_send(station_mac, bs, sizeof(espnow_sensor_data_t));

    xSemaphoreTake(espnow_send_lock, portMAX_DELAY);

    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();

    ESP_LOGI(LOG_TAG, "Running time: %lums, gonna sleep...", clock());
    esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000);
    esp_deep_sleep_start();
}