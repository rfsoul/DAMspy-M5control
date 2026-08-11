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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"

#include "M5GFX.h"

#include "hid_tunnel_protocol.h"


#define ESPNOW_CHANNEL         6
#define RESPONSE_TIMEOUT_MS    5000
#define TEST_INTERVAL_MS       3000
#define HID_READ_LENGTH        64
#define EMPTY_READ_TIMEOUT_MS  500
#define BRIDGE_RESPONSE_MARGIN_MS 2000
#define BRIDGE_OPERATION_TIMEOUT_MS 5000
#define SERIAL_INNER_MAX       (HID_TUNNEL_HEADER_SIZE + HID_TUNNEL_MAX_BODY)
#define SERIAL_RAW_MAX         (SERIAL_INNER_MAX + 2)
#define SERIAL_ENCODED_MAX     (SERIAL_RAW_MAX + (SERIAL_RAW_MAX / 254) + 1)
#define STICKS3_BUTTON_A_GPIO  GPIO_NUM_11


typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    int length;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} received_frame_t;


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

    if (reason[0] != '\0') {
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


static bool battery_write(
    uint32_t request_id,
    const char **reason
)
{
    received_frame_t received;
    esp_err_t err = exchange_operation(
        HID_TUNNEL_WRITE_REQUEST,
        request_id,
        test_frame,
        sizeof(test_frame),
        &received
    );

    if (err != ESP_OK) {
        *reason = err == ESP_ERR_TIMEOUT
            ? "WRITE TIMEOUT"
            : "WRITE ERROR";
        return false;
    }

    hid_tunnel_message_t response = {};

    if (
        !hid_tunnel_decode(
            received.data,
            received.length,
            &response
        ) ||
        response.type != HID_TUNNEL_WRITE_RESPONSE ||
        response.request_id != request_id ||
        response.body_length != HID_TUNNEL_WRITE_RESPONSE_SIZE ||
        response.body[0] != HID_TUNNEL_RESULT_OK ||
        hid_tunnel_get_u16(&response.body[1]) != sizeof(test_frame)
    ) {
        *reason = "WRITE ERROR";
        return false;
    }

    return true;
}


static bool hid_read(
    uint32_t request_id,
    uint32_t timeout_ms,
    uint8_t *result,
    uint8_t *data,
    size_t *data_length,
    const char **reason
)
{
    uint8_t read_request[HID_TUNNEL_READ_REQUEST_SIZE];
    hid_tunnel_put_u16(read_request, HID_READ_LENGTH);
    hid_tunnel_put_u32(&read_request[2], timeout_ms);

    received_frame_t received;
    esp_err_t err = exchange_operation(
        HID_TUNNEL_READ_REQUEST,
        request_id,
        read_request,
        sizeof(read_request),
        &received
    );

    if (err != ESP_OK) {
        *reason = err == ESP_ERR_TIMEOUT
            ? "READ LINK TIMEOUT"
            : "READ ERROR";
        return false;
    }

    hid_tunnel_message_t response = {};

    if (
        !hid_tunnel_decode(
            received.data,
            received.length,
            &response
        ) ||
        response.type != HID_TUNNEL_READ_RESPONSE ||
        response.request_id != request_id ||
        response.body_length < HID_TUNNEL_READ_RESPONSE_OVERHEAD
    ) {
        *reason = "READ ERROR";
        return false;
    }

    *result = response.body[0];
    *data_length = hid_tunnel_get_u16(&response.body[1]);

    if (
        response.body_length !=
            HID_TUNNEL_READ_RESPONSE_OVERHEAD +
            *data_length ||
        *data_length > HID_TUNNEL_MAX_HID_BYTES
    ) {
        *reason = "READ LENGTH";
        return false;
    }

    if (*data_length > 0) {
        memcpy(
            data,
            &response.body[HID_TUNNEL_READ_RESPONSE_OVERHEAD],
            *data_length
        );
    }

    return true;
}


static bool valid_battery_response(
    const uint8_t *data,
    size_t length
)
{
    return
        length >= 3 &&
        data[0] == 0x02 &&
        data[1] == 0x61 &&
        data[2] == 0x41;
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

static uint16_t crc16_ccitt_false(
    const uint8_t *data,
    size_t length
)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000)
                ? (uint16_t)((crc << 1) ^ 0x1021)
                : (uint16_t)(crc << 1);
        }
    }

    return crc;
}


static size_t cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity
)
{
    if (output_capacity == 0) {
        return 0;
    }

    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < input_length) {
        if (input[read_index] == 0) {
            if (code_index >= output_capacity) {
                return 0;
            }

            output[code_index] = code;
            code_index = write_index++;
            code = 1;
            read_index++;
        }
        else {
            if (write_index >= output_capacity) {
                return 0;
            }

            output[write_index++] = input[read_index++];
            code++;

            if (code == 0xFF) {
                if (code_index >= output_capacity) {
                    return 0;
                }

                output[code_index] = code;
                code_index = write_index++;
                code = 1;
            }
        }
    }

    if (code_index >= output_capacity) {
        return 0;
    }

    output[code_index] = code;
    return write_index;
}


static size_t cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity
)
{
    size_t read_index = 0;
    size_t write_index = 0;

    while (read_index < input_length) {
        uint8_t code = input[read_index++];

        if (code == 0) {
            return 0;
        }

        size_t copy_length = code - 1;

        if (
            read_index + copy_length > input_length ||
            write_index + copy_length > output_capacity
        ) {
            return 0;
        }

        memcpy(
            &output[write_index],
            &input[read_index],
            copy_length
        );
        read_index += copy_length;
        write_index += copy_length;

        if (code != 0xFF && read_index < input_length) {
            if (write_index >= output_capacity) {
                return 0;
            }

            output[write_index++] = 0;
        }
    }

    return write_index;
}


static bool valid_serial_request(
    const uint8_t *inner,
    size_t inner_length,
    hid_tunnel_message_t *message
)
{
    if (!hid_tunnel_decode(inner, inner_length, message)) {
        return false;
    }

    switch (message->type) {
        case HID_TUNNEL_WRITE_REQUEST:
            return
                message->body_length > 0 &&
                message->body_length <= HID_TUNNEL_MAX_HID_BYTES;

        case HID_TUNNEL_READ_REQUEST:
            return
                message->body_length == HID_TUNNEL_READ_REQUEST_SIZE &&
                hid_tunnel_get_u16(message->body) > 0 &&
                hid_tunnel_get_u16(message->body) <=
                    HID_TUNNEL_MAX_HID_BYTES &&
                hid_tunnel_get_u32(&message->body[2]) > 0;

        case HID_TUNNEL_STATUS_REQUEST:
            return message->body_length == 0;

        default:
            return false;
    }
}


static uint8_t expected_response_type(uint8_t request_type)
{
    return request_type + 1;
}


static uint32_t response_timeout_ms(
    const hid_tunnel_message_t *request
)
{
    if (request->type != HID_TUNNEL_READ_REQUEST) {
        return BRIDGE_OPERATION_TIMEOUT_MS;
    }

    uint32_t requested = hid_tunnel_get_u32(&request->body[2]);

    if (requested > UINT32_MAX - BRIDGE_RESPONSE_MARGIN_MS) {
        return UINT32_MAX;
    }

    return requested + BRIDGE_RESPONSE_MARGIN_MS;
}


static bool serial_write_all(
    const uint8_t *data,
    size_t length
)
{
    size_t written = 0;

    while (written < length) {
        int result = usb_serial_jtag_write_bytes(
            &data[written],
            length - written,
            pdMS_TO_TICKS(1000)
        );

        if (result <= 0) {
            return false;
        }

        written += result;
    }

    return true;
}


static bool serial_send_inner(
    const uint8_t *inner,
    size_t inner_length
)
{
    uint8_t raw[SERIAL_RAW_MAX];
    uint8_t encoded[SERIAL_ENCODED_MAX + 1];

    if (inner_length > SERIAL_INNER_MAX) {
        return false;
    }

    memcpy(raw, inner, inner_length);
    hid_tunnel_put_u16(
        &raw[inner_length],
        crc16_ccitt_false(inner, inner_length)
    );

    size_t encoded_length = cobs_encode(
        raw,
        inner_length + 2,
        encoded,
        SERIAL_ENCODED_MAX
    );

    if (encoded_length == 0) {
        return false;
    }

    encoded[encoded_length++] = 0;
    return serial_write_all(encoded, encoded_length);
}


static bool serial_read_inner(
    uint8_t *inner,
    size_t *inner_length
)
{
    static uint8_t encoded[SERIAL_ENCODED_MAX];
    static size_t encoded_length = 0;
    static bool overflow = false;
    uint8_t byte;

    while (1) {
        if (
            usb_serial_jtag_read_bytes(
                &byte,
                1,
                pdMS_TO_TICKS(100)
            ) != 1
        ) {
            continue;
        }

        if (byte != 0) {
            if (!overflow && encoded_length < sizeof(encoded)) {
                encoded[encoded_length++] = byte;
            }
            else {
                overflow = true;
            }

            continue;
        }

        if (overflow || encoded_length == 0) {
            overflow = false;
            encoded_length = 0;
            return false;
        }

        uint8_t raw[SERIAL_RAW_MAX];
        size_t raw_length = cobs_decode(
            encoded,
            encoded_length,
            raw,
            sizeof(raw)
        );
        encoded_length = 0;

        if (raw_length < HID_TUNNEL_HEADER_SIZE + 2) {
            return false;
        }

        size_t candidate_length = raw_length - 2;
        uint16_t supplied_crc = hid_tunnel_get_u16(
            &raw[candidate_length]
        );

        if (
            supplied_crc !=
                crc16_ccitt_false(raw, candidate_length)
        ) {
            return false;
        }

        memcpy(inner, raw, candidate_length);
        *inner_length = candidate_length;
        return true;
    }
}


static void show_bridge(
    uint32_t forwarded,
    uint32_t returned,
    uint32_t rejected,
    uint8_t type,
    uint32_t request_id,
    const char *state,
    bool error
)
{
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.setCursor(8, 6);
    display.println("DAMspy BRIDGE");
    display.printf(
        "TX:%lu RX:%lu BAD:%lu\n",
        (unsigned long)forwarded,
        (unsigned long)returned,
        (unsigned long)rejected
    );
    display.printf(
        "Type:%02X ID:%lu\n",
        type,
        (unsigned long)request_id
    );
    display.setTextColor(error ? TFT_RED : TFT_GREEN, TFT_BLACK);
    display.println(state);
}


static void run_bridge(void)
{
    usb_serial_jtag_driver_config_t serial_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    if (
        usb_serial_jtag_driver_install(&serial_config) != ESP_OK
    ) {
        show_bridge(0, 0, 1, 0, 0, "SERIAL ERROR", true);
        return;
    }

    uint32_t forwarded = 0;
    uint32_t returned = 0;
    uint32_t rejected = 0;
    show_bridge(0, 0, 0, 0, 0, "READY", false);

    while (1) {
        uint8_t inner[SERIAL_INNER_MAX];
        size_t inner_length = 0;

        if (!serial_read_inner(inner, &inner_length)) {
            rejected++;
            show_bridge(
                forwarded, returned, rejected,
                0, 0, "BAD SERIAL FRAME", true
            );
            continue;
        }

        hid_tunnel_message_t request = {};

        if (
            !valid_serial_request(
                inner,
                inner_length,
                &request
            )
        ) {
            rejected++;
            show_bridge(
                forwarded, returned, rejected,
                0, 0, "BAD REQUEST", true
            );
            continue;
        }

        xQueueReset(receive_queue);
        esp_err_t err = esp_now_send(
            broadcast_address,
            inner,
            inner_length
        );

        if (err != ESP_OK) {
            rejected++;
            show_bridge(
                forwarded, returned, rejected,
                request.type, request.request_id,
                "ESP-NOW SEND FAIL", true
            );
            continue;
        }

        forwarded++;
        show_bridge(
            forwarded, returned, rejected,
            request.type, request.request_id,
            "WAITING", false
        );

        int64_t deadline_us =
            esp_timer_get_time() +
            ((int64_t)response_timeout_ms(&request) * 1000);
        bool matched = false;

        while (esp_timer_get_time() < deadline_us) {
            int64_t remaining_us =
                deadline_us - esp_timer_get_time();
            TickType_t wait_ticks = pdMS_TO_TICKS(
                (remaining_us + 999) / 1000
            );

            if (wait_ticks == 0) {
                wait_ticks = 1;
            }

            received_frame_t received = {};

            if (
                xQueueReceive(
                    receive_queue,
                    &received,
                    wait_ticks
                ) != pdTRUE
            ) {
                break;
            }

            hid_tunnel_message_t response = {};

            if (
                !hid_tunnel_decode(
                    received.data,
                    received.length,
                    &response
                ) ||
                response.type !=
                    expected_response_type(request.type) ||
                response.request_id != request.request_id
            ) {
                continue;
            }

            matched = true;

            if (
                serial_send_inner(
                    received.data,
                    received.length
                )
            ) {
                returned++;
                show_bridge(
                    forwarded, returned, rejected,
                    request.type, request.request_id,
                    "RESPONSE SENT", false
                );
            }
            else {
                rejected++;
                show_bridge(
                    forwarded, returned, rejected,
                    request.type, request.request_id,
                    "SERIAL SEND FAIL", true
                );
            }

            break;
        }

        if (!matched) {
            rejected++;
            show_bridge(
                forwarded, returned, rejected,
                request.type, request.request_id,
                "LINK TIMEOUT", true
            );
        }
    }
}


static void run_diagnostic(void)
{
    show_starting();

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
        uint32_t first_id = ((test_count - 1) * 5) + 1;
        const char *reason = "";
        const char *pass_detail = "";
        int response_length = -1;
        uint8_t read_result = HID_TUNNEL_RESULT_USB_ERROR;
        uint8_t read_data[HID_TUNNEL_MAX_HID_BYTES];
        size_t read_length = 0;
        bool empty_read_timed_out = false;

        printf(
            "Battery WRITE 1: %lu\n",
            (unsigned long)first_id
        );

        bool test_ok = battery_write(first_id, &reason);

        if (test_ok) {
            printf(
                "Battery READ 1: %lu\n",
                (unsigned long)(first_id + 1)
            );

            test_ok = hid_read(
                first_id + 1,
                RESPONSE_TIMEOUT_MS,
                &read_result,
                read_data,
                &read_length,
                &reason
            );

            if (
                test_ok &&
                (
                    read_result != HID_TUNNEL_RESULT_OK ||
                    !valid_battery_response(read_data, read_length)
                )
            ) {
                reason = read_result == HID_TUNNEL_RESULT_TIMEOUT
                    ? "BATTERY TIMEOUT"
                    : "BAD PREFIX 1";
                test_ok = false;
            }
        }

        if (test_ok) {
            print_hex("Battery RX 1", read_data, read_length);
            read_length = 0;

            printf(
                "Empty READ: %lu (%d ms)\n",
                (unsigned long)(first_id + 2),
                EMPTY_READ_TIMEOUT_MS
            );

            test_ok = hid_read(
                first_id + 2,
                EMPTY_READ_TIMEOUT_MS,
                &read_result,
                read_data,
                &read_length,
                &reason
            );

            if (test_ok && read_result == HID_TUNNEL_RESULT_TIMEOUT) {
                empty_read_timed_out = true;
                printf("Empty READ returned TIMEOUT\n");
            }
            else if (test_ok && read_result == HID_TUNNEL_RESULT_OK) {
                printf("Empty READ returned real data\n");
                print_hex("Empty READ RX", read_data, read_length);
            }
            else if (test_ok) {
                reason = "EMPTY READ ERROR";
                test_ok = false;
            }
        }

        if (test_ok) {
            printf(
                "Battery WRITE 2: %lu\n",
                (unsigned long)(first_id + 3)
            );
            test_ok = battery_write(first_id + 3, &reason);
        }

        if (test_ok) {
            read_length = 0;
            printf(
                "Battery READ 2: %lu\n",
                (unsigned long)(first_id + 4)
            );

            test_ok = hid_read(
                first_id + 4,
                RESPONSE_TIMEOUT_MS,
                &read_result,
                read_data,
                &read_length,
                &reason
            );

            if (
                test_ok &&
                (
                    read_result != HID_TUNNEL_RESULT_OK ||
                    !valid_battery_response(read_data, read_length)
                )
            ) {
                reason = read_result == HID_TUNNEL_RESULT_TIMEOUT
                    ? "POST READ TIMEOUT"
                    : "BAD PREFIX 2";
                test_ok = false;
            }
        }

        if (test_ok) {
            print_hex("Battery RX 2", read_data, read_length);
            response_length = read_length;
            pass_detail = empty_read_timed_out
                ? "TIMEOUT RECOVERED"
                : "EMPTY READ HAD DATA";
        }

        if (test_ok) {
            pass_count++;
            printf("RESULT: PASS\n");
            show_result(
                test_count, pass_count, fail_count,
                response_length, true, pass_detail
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


extern "C" void app_main(void)
{
    if (!display.init()) {
        return;
    }

    display.setRotation(1);
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.setCursor(8, 12);
    display.println("DAMspy");
    display.println("Hold BtnA:");
    display.println("diagnostic");

    gpio_config_t button_config = {};
    button_config.pin_bit_mask =
        1ULL << STICKS3_BUTTON_A_GPIO;
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&button_config);

    bool diagnostic_mode = false;

    for (int i = 0; i < 40; i++) {
        diagnostic_mode |=
            gpio_get_level(STICKS3_BUTTON_A_GPIO) == 0;
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    esp_err_t err = initialise_espnow();

    if (err != ESP_OK) {
        show_result(0, 0, 1, -1, false, "ESPNOW SETUP");
        return;
    }

    if (diagnostic_mode) {
        run_diagnostic();
    }
    else {
        run_bridge();
    }
}
