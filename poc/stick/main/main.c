#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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


#define ESPNOW_CHANNEL         6
#define ECHO_TIMEOUT_MS        5000


typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    int length;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} received_frame_t;


static const char *TAG = "stick_echo_test";

static const uint8_t broadcast_address[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const uint8_t test_frame[] = {
    0x00, 0x01, 0x61, 0x7F,
    0x80, 0xA5, 0x5A, 0xFF,
    0x10, 0x20, 0x30, 0x40,
    0xDE, 0xAD, 0xBE, 0xEF
};

static QueueHandle_t receive_queue = NULL;


static void print_hex(
    const char *label,
    const uint8_t *data,
    int length
)
{
    printf("%s (%d bytes):", label, length);

    for (int i = 0; i < length; i++) {
        printf(" %02X", data[i]);
    }

    printf("\n");
}


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

    received_frame_t frame = {
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


static esp_err_t initialise_espnow(void)
{
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();

    if (err != ESP_OK) {
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
        1,
        sizeof(received_frame_t)
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

    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        broadcast_address,
        ESP_NOW_ETH_ALEN
    );

    peer.ifidx = WIFI_IF_STA;
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    return esp_now_add_peer(&peer);
}


void app_main(void)
{
    esp_err_t err = initialise_espnow();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ESP-NOW setup failed: %s",
            esp_err_to_name(err)
        );
        return;
    }

    uint8_t station_mac[ESP_NOW_ETH_ALEN];

    ESP_ERROR_CHECK(
        esp_wifi_get_mac(
            WIFI_IF_STA,
            station_mac
        )
    );

    printf(
        "M5StickS3 ESP-NOW echo test\n"
        "Channel: %d\n"
        "Station MAC: " MACSTR "\n",
        ESPNOW_CHANNEL,
        MAC2STR(station_mac)
    );

    print_hex(
        "TX",
        test_frame,
        sizeof(test_frame)
    );

    err = esp_now_send(
        broadcast_address,
        test_frame,
        sizeof(test_frame)
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "send failed: %s",
            esp_err_to_name(err)
        );
        printf("RESULT: FAIL\n");
        return;
    }

    received_frame_t echo;

    if (
        xQueueReceive(
            receive_queue,
            &echo,
            pdMS_TO_TICKS(ECHO_TIMEOUT_MS)
        ) != pdTRUE
    ) {
        printf("RX: timeout after %d ms\n", ECHO_TIMEOUT_MS);
        printf("RESULT: FAIL\n");
        return;
    }

    printf(
        "Echo source: " MACSTR "\n",
        MAC2STR(echo.source)
    );

    print_hex(
        "RX",
        echo.data,
        echo.length
    );

    bool matches =
        echo.length == sizeof(test_frame) &&
        memcmp(
            echo.data,
            test_frame,
            sizeof(test_frame)
        ) == 0;

    printf(
        "RESULT: %s\n",
        matches ? "PASS" : "FAIL"
    );
}
