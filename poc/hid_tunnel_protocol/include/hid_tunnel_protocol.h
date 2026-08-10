#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


#define HID_TUNNEL_MAGIC               0xD1
#define HID_TUNNEL_TYPE_REQUEST        0x01
#define HID_TUNNEL_TYPE_RESPONSE       0x02
#define HID_TUNNEL_HEADER_SIZE         8
#define HID_TUNNEL_MAX_PAYLOAD         242


static inline size_t hid_tunnel_encode(
    uint8_t *frame,
    size_t frame_capacity,
    uint8_t type,
    uint32_t transaction_id,
    const uint8_t *payload,
    size_t payload_length
)
{
    if (
        frame == NULL ||
        payload == NULL ||
        payload_length == 0 ||
        payload_length > HID_TUNNEL_MAX_PAYLOAD ||
        frame_capacity < HID_TUNNEL_HEADER_SIZE + payload_length
    ) {
        return 0;
    }

    frame[0] = HID_TUNNEL_MAGIC;
    frame[1] = type;
    frame[2] = transaction_id & 0xFF;
    frame[3] = (transaction_id >> 8) & 0xFF;
    frame[4] = (transaction_id >> 16) & 0xFF;
    frame[5] = (transaction_id >> 24) & 0xFF;
    frame[6] = payload_length & 0xFF;
    frame[7] = (payload_length >> 8) & 0xFF;

    for (size_t i = 0; i < payload_length; i++) {
        frame[HID_TUNNEL_HEADER_SIZE + i] = payload[i];
    }

    return HID_TUNNEL_HEADER_SIZE + payload_length;
}


static inline bool hid_tunnel_decode(
    const uint8_t *frame,
    size_t frame_length,
    uint8_t expected_type,
    uint32_t *transaction_id,
    const uint8_t **payload,
    size_t *payload_length
)
{
    if (
        frame == NULL ||
        transaction_id == NULL ||
        payload == NULL ||
        payload_length == NULL ||
        frame_length < HID_TUNNEL_HEADER_SIZE ||
        frame[0] != HID_TUNNEL_MAGIC ||
        frame[1] != expected_type
    ) {
        return false;
    }

    size_t decoded_length =
        (size_t)frame[6] |
        ((size_t)frame[7] << 8);

    if (
        decoded_length == 0 ||
        decoded_length > HID_TUNNEL_MAX_PAYLOAD ||
        frame_length != HID_TUNNEL_HEADER_SIZE + decoded_length
    ) {
        return false;
    }

    *transaction_id =
        (uint32_t)frame[2] |
        ((uint32_t)frame[3] << 8) |
        ((uint32_t)frame[4] << 16) |
        ((uint32_t)frame[5] << 24);

    *payload = &frame[HID_TUNNEL_HEADER_SIZE];
    *payload_length = decoded_length;

    return true;
}
