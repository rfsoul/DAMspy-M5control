#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define HID_TUNNEL_MAGIC                   0xD1
#define HID_TUNNEL_HEADER_SIZE             8
#define HID_TUNNEL_MAX_BODY                242
#define HID_TUNNEL_MAX_HID_BYTES           239

#define HID_TUNNEL_WRITE_REQUEST           0x01
#define HID_TUNNEL_WRITE_RESPONSE          0x02
#define HID_TUNNEL_READ_REQUEST            0x03
#define HID_TUNNEL_READ_RESPONSE           0x04
#define HID_TUNNEL_STATUS_REQUEST          0x05
#define HID_TUNNEL_STATUS_RESPONSE         0x06
#define HID_TUNNEL_NODE_DIAG_REQUEST       0x07
#define HID_TUNNEL_NODE_DIAG_RESPONSE      0x08
#define HID_TUNNEL_NODE_RESET_REQUEST      0x09
#define HID_TUNNEL_NODE_RESET_RESPONSE     0x0A
#define HID_TUNNEL_GATEWAY_DIAG_REQUEST    0x0B
#define HID_TUNNEL_GATEWAY_DIAG_RESPONSE   0x0C
#define HID_TUNNEL_GATEWAY_RESET_REQUEST   0x0D
#define HID_TUNNEL_GATEWAY_RESET_RESPONSE  0x0E
#define HID_TUNNEL_GATEWAY_SCAN_REQUEST     0x0F
#define HID_TUNNEL_GATEWAY_SCAN_RESPONSE    0x10

#define HID_TUNNEL_RESULT_OK               0x00
#define HID_TUNNEL_RESULT_TIMEOUT          0x01
#define HID_TUNNEL_RESULT_NO_DEVICE        0x02
#define HID_TUNNEL_RESULT_BUSY             0x03
#define HID_TUNNEL_RESULT_INVALID_REQUEST  0x04
#define HID_TUNNEL_RESULT_USB_ERROR        0x05

#define HID_TUNNEL_WRITE_RESPONSE_SIZE     3
#define HID_TUNNEL_READ_REQUEST_SIZE       6
#define HID_TUNNEL_READ_RESPONSE_OVERHEAD  3
#define HID_TUNNEL_STATUS_RESPONSE_SIZE    6

#define HID_TUNNEL_DIAG_VERSION            1
#define HID_TUNNEL_DIAG_FIXED_SIZE         37
#define HID_TUNNEL_DIAG_VERSION_MAX        31

#define HID_TUNNEL_DIAG_FLAG_USB_CONNECTED 0x01
#define HID_TUNNEL_DIAG_FLAG_HID_READY     0x02
#define HID_TUNNEL_DIAG_FLAG_BUSY          0x04
#define HID_TUNNEL_DIAG_FLAG_RX_VALID      0x08
#define HID_TUNNEL_DIAG_FLAG_PREV_VALID    0x10

#define HID_TUNNEL_STATE_BOOTING            0
#define HID_TUNNEL_STATE_READY              1
#define HID_TUNNEL_STATE_WAITING            2
#define HID_TUNNEL_STATE_USB_OPERATION      3
#define HID_TUNNEL_STATE_OTA                4
#define HID_TUNNEL_STATE_ERROR              5
#define HID_TUNNEL_STATE_RESETTING          6

typedef struct {
    uint8_t type;
    uint32_t request_id;
    const uint8_t *body;
    size_t body_length;
} hid_tunnel_message_t;

static inline uint16_t hid_tunnel_get_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline uint32_t hid_tunnel_get_u32(const uint8_t *data)
{
    return
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static inline void hid_tunnel_put_u16(uint8_t *data, uint16_t value)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

static inline void hid_tunnel_put_u32(uint8_t *data, uint32_t value)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
}

static inline size_t hid_tunnel_encode(
    uint8_t *frame,
    size_t frame_capacity,
    uint8_t type,
    uint32_t request_id,
    const uint8_t *body,
    size_t body_length
)
{
    if (
        frame == NULL ||
        body_length > HID_TUNNEL_MAX_BODY ||
        (body_length > 0 && body == NULL) ||
        frame_capacity < HID_TUNNEL_HEADER_SIZE + body_length
    ) {
        return 0;
    }

    frame[0] = HID_TUNNEL_MAGIC;
    frame[1] = type;
    hid_tunnel_put_u32(&frame[2], request_id);
    hid_tunnel_put_u16(&frame[6], body_length);

    if (body_length > 0) {
        memcpy(&frame[HID_TUNNEL_HEADER_SIZE], body, body_length);
    }

    return HID_TUNNEL_HEADER_SIZE + body_length;
}

static inline bool hid_tunnel_decode(
    const uint8_t *frame,
    size_t frame_length,
    hid_tunnel_message_t *message
)
{
    if (
        frame == NULL ||
        message == NULL ||
        frame_length < HID_TUNNEL_HEADER_SIZE ||
        frame[0] != HID_TUNNEL_MAGIC
    ) {
        return false;
    }

    size_t body_length = hid_tunnel_get_u16(&frame[6]);

    if (
        body_length > HID_TUNNEL_MAX_BODY ||
        frame_length != HID_TUNNEL_HEADER_SIZE + body_length
    ) {
        return false;
    }

    message->type = frame[1];
    message->request_id = hid_tunnel_get_u32(&frame[2]);
    message->body = &frame[HID_TUNNEL_HEADER_SIZE];
    message->body_length = body_length;
    return true;
}
