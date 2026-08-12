#pragma once

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_now.h"

#include "hid_tunnel_protocol.h"


typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    uint8_t magic;
    uint8_t type;
    uint32_t request_id;
    size_t body_length;
    uint8_t body[HID_TUNNEL_MAX_BODY];
} espnow_hid_message_t;


esp_err_t espnow_transport_start(void);

bool espnow_transport_get_last_rssi(int8_t *rssi);
bool espnow_transport_get_last_rx(
    int8_t *rssi,
    uint32_t *age_seconds
);

bool espnow_transport_receive_message(
    espnow_hid_message_t *message,
    TickType_t wait_ticks
);

esp_err_t espnow_transport_send_message(
    const uint8_t destination[ESP_NOW_ETH_ALEN],
    uint8_t type,
    uint32_t request_id,
    const uint8_t *body,
    size_t body_length
);

esp_err_t espnow_transport_send_ota_message(
    const uint8_t destination[ESP_NOW_ETH_ALEN], uint8_t type,
    uint32_t request_id, const uint8_t *body, size_t body_length
);
