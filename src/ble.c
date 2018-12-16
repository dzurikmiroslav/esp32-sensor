#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_gatts_api.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include "ble.h"

#define LOG_TAG "BLE"

esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "GAP handle event: %d", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    {
        ESP_LOGD(LOG_TAG, "Start advertising");
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
        break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGD(LOG_TAG, "Stoping advertising");
        ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGD(LOG_TAG, "Advertising done");
        break;
    default:
        break;
    }
}

esp_err_t ble_init()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Controller mem release failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

void set_beacon_data(uint8_t *raw_data, uint32_t raw_data_len)
{
    esp_ble_gap_config_adv_data_raw(raw_data, raw_data_len);
}