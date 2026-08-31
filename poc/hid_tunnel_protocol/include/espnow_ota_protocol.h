#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hid_tunnel_protocol.h"

#define ESPNOW_OTA_MAGIC 0xD3
#define ESPNOW_OTA_HEADER_SIZE HID_TUNNEL_HEADER_SIZE
#define ESPNOW_OTA_MAX_BODY HID_TUNNEL_MAX_BODY
#define ESPNOW_OTA_SHA256_SIZE 32
#define ESPNOW_OTA_BEGIN_BODY_SIZE (4 + ESPNOW_OTA_SHA256_SIZE)
#define ESPNOW_OTA_DATA_OVERHEAD 4
#define ESPNOW_OTA_MAX_CHUNK (ESPNOW_OTA_MAX_BODY - ESPNOW_OTA_DATA_OVERHEAD)
#define ESPNOW_OTA_BEGIN_REQUEST 0x01
#define ESPNOW_OTA_BEGIN_RESPONSE 0x02
#define ESPNOW_OTA_DATA_REQUEST 0x03
#define ESPNOW_OTA_DATA_RESPONSE 0x04
#define ESPNOW_OTA_END_REQUEST 0x05
#define ESPNOW_OTA_END_RESPONSE 0x06
#define ESPNOW_OTA_ABORT_REQUEST 0x07
#define ESPNOW_OTA_ABORT_RESPONSE 0x08
#define ESPNOW_OTA_STATUS_REQUEST 0x09
#define ESPNOW_OTA_STATUS_RESPONSE 0x0A
#define ESPNOW_OTA_RESULT_OK 0x00
#define ESPNOW_OTA_RESULT_BUSY 0x01
#define ESPNOW_OTA_RESULT_INVALID 0x02
#define ESPNOW_OTA_RESULT_OFFSET 0x03
#define ESPNOW_OTA_RESULT_WRITE_ERROR 0x04
#define ESPNOW_OTA_RESULT_VERIFY_ERROR 0x05
#define ESPNOW_OTA_RESULT_NO_SESSION 0x06
#define ESPNOW_OTA_ACK_BODY_SIZE 5
#define ESPNOW_OTA_VERSION_MAX 31

typedef struct {
    uint8_t type;
    uint32_t request_id;
    const uint8_t *body;
    size_t body_length;
} espnow_ota_message_t;

static inline size_t espnow_ota_encode(uint8_t *frame, size_t frame_capacity,
    uint8_t type, uint32_t request_id, const uint8_t *body, size_t body_length)
{
    if (frame == NULL || body_length > ESPNOW_OTA_MAX_BODY ||
        (body_length > 0 && body == NULL) ||
        frame_capacity < ESPNOW_OTA_HEADER_SIZE + body_length) return 0;
    frame[0] = ESPNOW_OTA_MAGIC;
    frame[1] = type;
    hid_tunnel_put_u32(&frame[2], request_id);
    hid_tunnel_put_u16(&frame[6], body_length);
    if (body_length > 0) memcpy(&frame[ESPNOW_OTA_HEADER_SIZE], body, body_length);
    return ESPNOW_OTA_HEADER_SIZE + body_length;
}

static inline bool espnow_ota_decode(const uint8_t *frame, size_t frame_length,
    espnow_ota_message_t *message)
{
    if (frame == NULL || message == NULL || frame_length < ESPNOW_OTA_HEADER_SIZE ||
        frame[0] != ESPNOW_OTA_MAGIC) return false;
    size_t body_length = hid_tunnel_get_u16(&frame[6]);
    if (body_length > ESPNOW_OTA_MAX_BODY ||
        frame_length != ESPNOW_OTA_HEADER_SIZE + body_length) return false;
    message->type = frame[1];
    message->request_id = hid_tunnel_get_u32(&frame[2]);
    message->body = &frame[ESPNOW_OTA_HEADER_SIZE];
    message->body_length = body_length;
    return true;
}
