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

#include "M5GFX.h"


#define ESPNOW_CHANNEL         6
#define ECHO_TIMEOUT_MS        5000
#define TEST_INTERVAL_MS       3000


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
static M5GFX display;


static void show_starting(void)
{
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.setCursor(8, 12);
    display.println("DAMspy ESP-NOW");
    display.println();
    display.println("Starting...");
}


static void show_result(
    uint32_t test_count,
    uint32_t pass_count,
    uint32_t fail_count,
    int received_length,
    bool passed,
    const char *reason
)
{
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.setCursor(8, 6);

    display.println("DAMspy ESP-NOW");
    display.printf(
        "Test:%lu P:%lu F:%lu\n",
        (unsigned long)test_count,
        (unsigned long)pass_count,
        (unsigned long)fail_count
    );
    display.printf("TX: %u bytes\n", (unsigned)sizeof(test_frame));

    if (received_length >= 0) {
        display.printf("RX: %d bytes\n", received_length);
    }
    else {
        display.println("RX: --");
    }

    display.setTextColor(
        passed ? TFT_GREEN : TFT_RED,
        TFT_BLACK
    );
    display.printf("RESULT: %s\n", passed ? "PASS" : "FAIL");

    if (!passed) {
        display.println(reason);
    }
}


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

    received_frame_t frame = {};

    frame.length = data_length;

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

    esp_now_peer_info_t peer = {};

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


extern "C" void app_main(void)
{
    if (!display.init()) {
        printf("Display initialization failed\n");
        return;
    }

    display.setRotation(1);
    show_starting();

    esp_err_t err = initialise_espnow();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ESP-NOW setup failed: %s",
            esp_err_to_name(err)
        );
        show_result(0, 0, 1, -1, false, "ESPNOW SETUP");
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

    vTaskDelay(pdMS_TO_TICKS(3000));

    uint32_t test_count = 0;
    uint32_t pass_count = 0;
    uint32_t fail_count = 0;

    while (1) {
        test_count++;
        xQueueReset(receive_queue);

        print_hex("TX", test_frame, sizeof(test_frame));

        err = esp_now_send(
            broadcast_address,
            test_frame,
            sizeof(test_frame)
        );

        if (err != ESP_OK) {
            fail_count++;
            ESP_LOGE(TAG, "send failed: %s", esp_err_to_name(err));
            printf("RESULT: FAIL (SEND)\n");
            show_result(
                test_count, pass_count, fail_count,
                -1, false, "SEND"
            );
            vTaskDelay(pdMS_TO_TICKS(TEST_INTERVAL_MS));
            continue;
        }

        received_frame_t echo;

        if (
            xQueueReceive(
                receive_queue,
                &echo,
                pdMS_TO_TICKS(ECHO_TIMEOUT_MS)
            ) != pdTRUE
        ) {
            fail_count++;
            printf("RX: timeout after %d ms\n", ECHO_TIMEOUT_MS);
            printf("RESULT: FAIL (TIMEOUT)\n");
            show_result(
                test_count, pass_count, fail_count,
                -1, false, "TIMEOUT"
            );
            vTaskDelay(pdMS_TO_TICKS(TEST_INTERVAL_MS));
            continue;
        }

        printf("Echo source: " MACSTR "\n", MAC2STR(echo.source));
        print_hex("RX", echo.data, echo.length);

        bool length_matches =
            echo.length == sizeof(test_frame);

        bool data_matches =
            length_matches &&
            memcmp(
                echo.data,
                test_frame,
                sizeof(test_frame)
            ) == 0;

        if (data_matches) {
            pass_count++;
            printf("RESULT: PASS\n");
            show_result(
                test_count, pass_count, fail_count,
                echo.length, true, ""
            );
        }
        else {
            fail_count++;
            const char *reason =
                length_matches
                ? "DATA MISMATCH"
                : "LENGTH";

            printf("RESULT: FAIL (%s)\n", reason);
            show_result(
                test_count, pass_count, fail_count,
                echo.length, false, reason
            );
        }

        vTaskDelay(pdMS_TO_TICKS(TEST_INTERVAL_MS));
    }
}
