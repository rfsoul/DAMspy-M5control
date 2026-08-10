#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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


static const char *TAG = "espnow_transport";

static QueueHandle_t receive_queue = NULL;


/* Wi-Fi task context: validate, copy, and queue only. */
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

    memcpy(frame.source, info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(frame.data, data, data_length);

    (void)xQueueSend(receive_queue, &frame, 0);
}


static esp_err_t ensure_peer(
    const uint8_t peer_address[ESP_NOW_ETH_ALEN]
)
{
    if (esp_now_is_peer_exist(peer_address)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr, peer_address, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    return esp_now_add_peer(&peer);
}


bool espnow_transport_receive_request(
    espnow_hid_request_t *request,
    TickType_t wait_ticks
)
{
    if (request == NULL || receive_queue == NULL) {
        return false;
    }

    espnow_frame_t frame;

    if (
        xQueueReceive(
            receive_queue,
            &frame,
            wait_ticks
        ) != pdTRUE
    ) {
        return false;
    }

    const uint8_t *payload = NULL;
    size_t payload_length = 0;
    uint32_t transaction_id = 0;

    if (
        !hid_tunnel_decode(
            frame.data,
            frame.length,
            HID_TUNNEL_TYPE_REQUEST,
            &transaction_id,
            &payload,
            &payload_length
        )
    ) {
        ESP_LOGW(TAG, "discarding invalid request frame");
        return false;
    }

    memcpy(request->source, frame.source, ESP_NOW_ETH_ALEN);
    request->transaction_id = transaction_id;
    request->payload_length = payload_length;
    memcpy(request->payload, payload, payload_length);

    return true;
}


esp_err_t espnow_transport_send_response(
    const uint8_t destination[ESP_NOW_ETH_ALEN],
    uint32_t transaction_id,
    const uint8_t *payload,
    size_t payload_length
)
{
    uint8_t frame[ESP_NOW_MAX_DATA_LEN];

    size_t frame_length = hid_tunnel_encode(
        frame,
        sizeof(frame),
        HID_TUNNEL_TYPE_RESPONSE,
        transaction_id,
        payload,
        payload_length
    );

    if (frame_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ensure_peer(destination);

    if (err != ESP_OK) {
        return err;
    }

    return esp_now_send(
        destination,
        frame,
        frame_length
    );
}


esp_err_t espnow_transport_start(void)
{
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

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

    err = esp_now_register_recv_cb(espnow_receive_callback);

    if (err != ESP_OK) {
        return err;
    }

    uint8_t station_mac[ESP_NOW_ETH_ALEN];

    err = esp_wifi_get_mac(WIFI_IF_STA, station_mac);

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
