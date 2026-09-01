#include "core_usb_proxy.h"

#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"

#include "hid_tunnel_protocol.h"

#define PROXY_LINE_MAX  640
#define PROXY_TIMEOUT_MS 12000

static bool proxy_started;
static char rx_line[PROXY_LINE_MAX];
static size_t rx_length;

static char hex_digit(uint8_t value)
{
    return value < 10 ? (char)('0' + value) : (char)('a' + value - 10);
}

static int hex_value(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

bool core_usb_proxy_start(void)
{
    if (proxy_started) return true;
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    if (usb_serial_jtag_driver_install(&config) != ESP_OK) return false;
    proxy_started = true;
    return true;
}

static bool write_request(const espnow_hid_message_t *request)
{
    char line[PROXY_LINE_MAX];
    int used = snprintf(
        line, sizeof(line), "@M5PX1,REQ,%08lx,%02x,",
        (unsigned long)request->request_id, request->type);
    if (used < 0 || (size_t)used + request->body_length * 2 + 2 > sizeof(line)) {
        return false;
    }
    for (size_t index = 0; index < request->body_length; ++index) {
        line[used++] = hex_digit(request->body[index] >> 4);
        line[used++] = hex_digit(request->body[index] & 0x0f);
    }
    line[used++] = '\n';
    return usb_serial_jtag_write_bytes(line, used, pdMS_TO_TICKS(100)) == used;
}

static bool parse_response(
    const char *line,
    uint32_t expected_id,
    uint8_t *response_type,
    uint8_t *body,
    size_t *body_length)
{
    const char *prefix = strstr(line, "@M5PX1,RSP,");
    if (prefix == NULL) return false;
    line = prefix;
    unsigned long request_id;
    unsigned int type;
    int consumed = 0;
    if (sscanf(line, "@M5PX1,RSP,%8lx,%2x,%n", &request_id, &type, &consumed) != 2 ||
        request_id != expected_id || type > 0xff || consumed <= 0) {
        return false;
    }
    size_t hex_length = strcspn(line + consumed, "\r\n");
    if ((hex_length & 1) != 0 || hex_length / 2 > HID_TUNNEL_MAX_BODY) return false;
    for (size_t index = 0; index < hex_length / 2; ++index) {
        int high = hex_value(line[consumed + index * 2]);
        int low = hex_value(line[consumed + index * 2 + 1]);
        if (high < 0 || low < 0) return false;
        body[index] = (uint8_t)((high << 4) | low);
    }
    *response_type = (uint8_t)type;
    *body_length = hex_length / 2;
    return true;
}

static bool read_response(
    uint32_t request_id,
    uint8_t *response_type,
    uint8_t *body,
    size_t *body_length)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(PROXY_TIMEOUT_MS);
    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        uint8_t incoming[128];
        int count = usb_serial_jtag_read_bytes(incoming, sizeof(incoming), pdMS_TO_TICKS(25));
        for (int index = 0; index < count; ++index) {
            char value = (char)incoming[index];
            if (value == '\n') {
                rx_line[rx_length] = '\0';
                bool matched = parse_response(
                    rx_line, request_id, response_type, body, body_length);
                rx_length = 0;
                if (matched) return true;
            } else if (value != '\r') {
                if (rx_length + 1 < sizeof(rx_line)) rx_line[rx_length++] = value;
                else rx_length = 0;
            }
        }
    }
    return false;
}

bool core_usb_proxy_handle(const espnow_hid_message_t *request)
{
    uint8_t expected_response;
    switch (request->type) {
        case HID_TUNNEL_WRITE_REQUEST: expected_response = HID_TUNNEL_WRITE_RESPONSE; break;
        case HID_TUNNEL_READ_REQUEST: expected_response = HID_TUNNEL_READ_RESPONSE; break;
        case HID_TUNNEL_STATUS_REQUEST: expected_response = HID_TUNNEL_STATUS_RESPONSE; break;
        default: return false;
    }

    uint8_t response_type = 0;
    uint8_t response_body[HID_TUNNEL_MAX_BODY];
    size_t response_length = 0;
    bool exchanged = proxy_started && write_request(request) && read_response(
        request->request_id, &response_type, response_body, &response_length);

    if (!exchanged || response_type != expected_response) {
        response_type = expected_response;
        if (request->type == HID_TUNNEL_STATUS_REQUEST) {
            memset(response_body, 0, HID_TUNNEL_STATUS_RESPONSE_SIZE);
            response_length = HID_TUNNEL_STATUS_RESPONSE_SIZE;
        } else {
            response_body[0] = HID_TUNNEL_RESULT_USB_ERROR;
            hid_tunnel_put_u16(&response_body[1], 0);
            response_length = HID_TUNNEL_READ_RESPONSE_OVERHEAD;
        }
    }

    (void)espnow_transport_send_message(
        request->source, response_type, request->request_id,
        response_body, response_length);
    return true;
}
