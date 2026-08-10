#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "espnow_echo.h"


#define ESPNOW_CHANNEL         6
#define ESPNOW_QUEUE_LENGTH    4


typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    int length;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} espnow_frame_t;


static const char *TAG = "espnow_echo";

static QueueHandle_t receive_queue = NULL;


/*
 * Runs in the Wi-Fi task. Copy only; peer management and transmission
 * are deliberately left to espnow_echo_task().
 */
static void espnow_receive_callback(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int data_length
)
{
    if (
        receive_queue == NULL ||
        info == NULL ||
        data == NULL ||
        data_length <= 0 ||
        data_length > ESP_NOW_MAX_DATA_LEN
    ) {
        return;
    }

    espnow_frame_t frame = {
        .length = data_length
    };

    memcpy(
        frame.source,
        info->src_addr,
        ESP_NOW_ETH_ALEN
    );

    memcpy(
        frame.data,
        data,
        data_length
    );

    (void)xQueueSend(
        receive_queue,
        &frame,
        0
    );
}


static esp_err_t ensure_peer(
    const uint8_t peer_address[ESP_NOW_ETH_ALEN]
)
{
    if (esp_now_is_peer_exist(peer_address)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        peer_address,
        ESP_NOW_ETH_ALEN
    );

    peer.ifidx = WIFI_IF_STA;
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    return esp_now_add_peer(&peer);
}


static void espnow_echo_task(void *arg)
{
    espnow_frame_t frame;

    while (1) {
        if (
            xQueueReceive(
                receive_queue,
                &frame,
                portMAX_DELAY
            ) != pdTRUE
        ) {
            continue;
        }

        esp_err_t err =
            ensure_peer(frame.source);

        if (err == ESP_OK) {
            err = esp_now_send(
                frame.source,
                frame.data,
                frame.length
            );
        }

        if (err == ESP_OK) {
            ESP_LOGI(
                TAG,
                "echoed %d opaque bytes to " MACSTR,
                frame.length,
                MAC2STR(frame.source)
            );
        }
        else {
            ESP_LOGE(
                TAG,
                "echo to " MACSTR " failed: %s",
                MAC2STR(frame.source),
                esp_err_to_name(err)
            );
        }
    }
}


esp_err_t espnow_echo_start(void)
{
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();

    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    ) {
        return err;
    }

    err = esp_event_loop_create_default();

    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    ) {
        return err;
    }

    wifi_init_config_t wifi_config =
        WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&wifi_config);

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_channel(
        ESPNOW_CHANNEL,
        WIFI_SECOND_CHAN_NONE
    );

    if (err != ESP_OK) {
        return err;
    }

    receive_queue = xQueueCreate(
        ESPNOW_QUEUE_LENGTH,
        sizeof(espnow_frame_t)
    );

    if (receive_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = esp_now_init();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_now_register_recv_cb(
        espnow_receive_callback
    );

    if (err != ESP_OK) {
        return err;
    }

    BaseType_t task_created = xTaskCreate(
        espnow_echo_task,
        "espnow_echo",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t station_mac[ESP_NOW_ETH_ALEN];

    err = esp_wifi_get_mac(
        WIFI_IF_STA,
        station_mac
    );

    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(
        TAG,
        "ready on channel %d, station MAC " MACSTR,
        ESPNOW_CHANNEL,
        MAC2STR(station_mac)
    );

    return ESP_OK;
}
