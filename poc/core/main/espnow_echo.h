#pragma once

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_now.h"

#include "hid_tunnel_protocol.h"


typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    uint32_t transaction_id;
    size_t payload_length;
    uint8_t payload[HID_TUNNEL_MAX_PAYLOAD];
} espnow_hid_request_t;


esp_err_t espnow_transport_start(void);

bool espnow_transport_receive_request(
    espnow_hid_request_t *request,
    TickType_t wait_ticks
);

esp_err_t espnow_transport_send_response(
    const uint8_t destination[ESP_NOW_ETH_ALEN],
    uint32_t transaction_id,
    const uint8_t *payload,
    size_t payload_length
);
