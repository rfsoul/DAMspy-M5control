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

#include "hid_tunnel_protocol.h"


#define ESPNOW_CHANNEL         6
#define RESPONSE_TIMEOUT_MS    5000
#define TEST_INTERVAL_MS       3000
#define HID_READ_LENGTH        64


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
    0x01, 0x61,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
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


static esp_err_t exchange_operation(
    uint8_t type,
    uint32_t request_id,
    const uint8_t *body,
    size_t body_length,
    received_frame_t *received
)
{
    uint8_t frame[ESP_NOW_MAX_DATA_LEN];
    size_t frame_length = hid_tunnel_encode(
        frame,
        sizeof(frame),
        type,
        request_id,
        body,
        body_length
    );

    if (frame_length == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = esp_now_send(
        broadcast_address,
        frame,
        frame_length
    );

    if (err != ESP_OK) {
        return err;
    }

    if (
        xQueueReceive(
            receive_queue,
            received,
            pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)
        ) != pdTRUE
    ) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
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
        "M5StickS3 HID tunnel test\n"
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

        print_hex("TX HID", test_frame, sizeof(test_frame));
        uint32_t write_id = (test_count * 2) - 1;
        uint32_t read_id = write_id + 1;
        received_frame_t received;
        const char *reason = "";
        int response_length = -1;

        printf("WRITE request: %lu\n", (unsigned long)write_id);
        err = exchange_operation(
            HID_TUNNEL_WRITE_REQUEST,
            write_id,
            test_frame,
            sizeof(test_frame),
            &received
        );

        hid_tunnel_message_t response = {};
        bool write_ok =
            err == ESP_OK &&
            hid_tunnel_decode(
                received.data,
                received.length,
                &response
            ) &&
            response.type == HID_TUNNEL_WRITE_RESPONSE &&
            response.request_id == write_id &&
            response.body_length == HID_TUNNEL_WRITE_RESPONSE_SIZE &&
            response.body[0] == HID_TUNNEL_RESULT_OK &&
            hid_tunnel_get_u16(&response.body[1]) == sizeof(test_frame);

        if (!write_ok) {
            reason = err == ESP_ERR_TIMEOUT
                ? "WRITE TIMEOUT"
                : "WRITE ERROR";
        }
        else {
            printf(
                "WRITE OK: %u bytes\n",
                hid_tunnel_get_u16(&response.body[1])
            );

            uint8_t read_request[HID_TUNNEL_READ_REQUEST_SIZE];
            hid_tunnel_put_u16(read_request, HID_READ_LENGTH);
            hid_tunnel_put_u32(&read_request[2], RESPONSE_TIMEOUT_MS);

            printf("READ request: %lu\n", (unsigned long)read_id);
            err = exchange_operation(
                HID_TUNNEL_READ_REQUEST,
                read_id,
                read_request,
                sizeof(read_request),
                &received
            );

            response = {};
            bool read_frame_ok =
                err == ESP_OK &&
                hid_tunnel_decode(
                    received.data,
                    received.length,
                    &response
                ) &&
                response.type == HID_TUNNEL_READ_RESPONSE &&
                response.request_id == read_id &&
                response.body_length >= HID_TUNNEL_READ_RESPONSE_OVERHEAD;

            if (!read_frame_ok) {
                reason = err == ESP_ERR_TIMEOUT
                    ? "READ TIMEOUT"
                    : "READ ERROR";
            }
            else if (response.body[0] != HID_TUNNEL_RESULT_OK) {
                reason = response.body[0] == HID_TUNNEL_RESULT_TIMEOUT
                    ? "READ TIMEOUT"
                    : "READ USB ERROR";
            }
            else {
                response_length = hid_tunnel_get_u16(&response.body[1]);

                if (
                    response.body_length !=
                        HID_TUNNEL_READ_RESPONSE_OVERHEAD +
                        response_length
                ) {
                    reason = "READ LENGTH";
                    response_length = -1;
                }
                else {
                    print_hex(
                        "RX HID",
                        &response.body[HID_TUNNEL_READ_RESPONSE_OVERHEAD],
                        response_length
                    );

                    if (
                        response_length < 3 ||
                        response.body[3] != 0x02 ||
                        response.body[4] != 0x61 ||
                        response.body[5] != 0x41
                    ) {
                        reason = "BAD PREFIX";
                    }
                }
            }
        }

        if (reason[0] == '\0') {
            pass_count++;
            printf("RESULT: PASS\n");
            show_result(
                test_count, pass_count, fail_count,
                response_length, true, ""
            );
        }
        else {
            fail_count++;
            printf("RESULT: FAIL (%s)\n", reason);
            show_result(
                test_count, pass_count, fail_count,
                response_length, false, reason
            );
        }

        vTaskDelay(pdMS_TO_TICKS(TEST_INTERVAL_MS));
    }
}
