#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "psa/crypto.h"

#include "core_ota.h"
#include "espnow_ota_protocol.h"

#define OTA_SESSION_TIMEOUT_US (180LL * 1000 * 1000)
#define OTA_RESTART_DELAY_MS 750

static const char *TAG = "core_ota";
static bool active;
static esp_ota_handle_t handle;
static const esp_partition_t *partition;
static uint32_t image_size;
static uint32_t received_size;
static uint8_t expected_sha256[ESPNOW_OTA_SHA256_SIZE];
static uint8_t owner[ESP_NOW_ETH_ALEN];
static int64_t last_activity_us;
static psa_hash_operation_t sha256 = PSA_HASH_OPERATION_INIT;
static bool sha256_initialised;

static void reset_session(bool abort_update)
{
    if (active && abort_update) (void)esp_ota_abort(handle);
    if (sha256_initialised) {
        (void)psa_hash_abort(&sha256);
        sha256 = psa_hash_operation_init();
        sha256_initialised = false;
    }
    active = false;
    handle = 0;
    partition = NULL;
    image_size = 0;
    received_size = 0;
    memset(owner, 0, sizeof(owner));
}

static void send_ack(const espnow_hid_message_t *message, uint8_t response_type,
    uint8_t result, uint32_t next_offset)
{
    uint8_t body[ESPNOW_OTA_ACK_BODY_SIZE];
    body[0] = result;
    hid_tunnel_put_u32(&body[1], next_offset);
    esp_err_t err = espnow_transport_send_ota_message(
        message->source, response_type, message->request_id, body, sizeof(body));
    if (err != ESP_OK) ESP_LOGE(TAG, "ack send: %s", esp_err_to_name(err));
}

static bool owner_matches(const espnow_hid_message_t *message)
{
    return memcmp(owner, message->source, ESP_NOW_ETH_ALEN) == 0;
}

static void send_status(const espnow_hid_message_t *message)
{
    const esp_app_desc_t *description = esp_app_get_description();
    size_t version_length = strnlen(description->version, ESPNOW_OTA_VERSION_MAX);
    uint8_t body[11 + ESPNOW_OTA_VERSION_MAX];
    body[0] = ESPNOW_OTA_RESULT_OK;
    body[1] = active ? 1 : 0;
    hid_tunnel_put_u32(&body[2], received_size);
    hid_tunnel_put_u32(&body[6], image_size);
    body[10] = (uint8_t)version_length;
    memcpy(&body[11], description->version, version_length);
    (void)espnow_transport_send_ota_message(message->source,
        ESPNOW_OTA_STATUS_RESPONSE, message->request_id,
        body, 11 + version_length);
}

bool core_ota_handle_message(const espnow_hid_message_t *message, bool operation_busy)
{
    if (message == NULL || message->magic != ESPNOW_OTA_MAGIC) return false;

    if (message->type == ESPNOW_OTA_STATUS_REQUEST) {
        send_status(message);
        return true;
    }

    if (message->type == ESPNOW_OTA_ABORT_REQUEST) {
        uint8_t result = ESPNOW_OTA_RESULT_NO_SESSION;
        uint32_t offset = received_size;
        if (active && owner_matches(message)) {
            reset_session(true);
            result = ESPNOW_OTA_RESULT_OK;
            offset = 0;
        }
        send_ack(message, ESPNOW_OTA_ABORT_RESPONSE, result, offset);
        return true;
    }

    if (message->type == ESPNOW_OTA_BEGIN_REQUEST) {
        if (message->body_length != ESPNOW_OTA_BEGIN_BODY_SIZE) {
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE, ESPNOW_OTA_RESULT_INVALID, 0);
            return true;
        }
        uint32_t requested_size = hid_tunnel_get_u32(message->body);
        if (active && owner_matches(message) && requested_size == image_size &&
            memcmp(&message->body[4], expected_sha256, ESPNOW_OTA_SHA256_SIZE) == 0) {
            last_activity_us = esp_timer_get_time();
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE, ESPNOW_OTA_RESULT_OK,
                received_size);
            return true;
        }
        if (active || operation_busy) {
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE, ESPNOW_OTA_RESULT_BUSY,
                received_size);
            return true;
        }
        const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
        if (requested_size == 0 || next == NULL || requested_size > next->size) {
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE, ESPNOW_OTA_RESULT_INVALID, 0);
            return true;
        }

        esp_err_t err = esp_ota_begin(next, requested_size, &handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "begin: %s", esp_err_to_name(err));
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE,
                ESPNOW_OTA_RESULT_WRITE_ERROR, 0);
            return true;
        }
        sha256 = psa_hash_operation_init();
        if (psa_crypto_init() != PSA_SUCCESS ||
            psa_hash_setup(&sha256, PSA_ALG_SHA_256) != PSA_SUCCESS) {
            (void)esp_ota_abort(handle);
            (void)psa_hash_abort(&sha256);
            send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE,
                ESPNOW_OTA_RESULT_WRITE_ERROR, 0);
            return true;
        }
        sha256_initialised = true;
        active = true;
        partition = next;
        image_size = requested_size;
        received_size = 0;
        memcpy(expected_sha256, &message->body[4], ESPNOW_OTA_SHA256_SIZE);
        memcpy(owner, message->source, ESP_NOW_ETH_ALEN);
        last_activity_us = esp_timer_get_time();
        ESP_LOGI(TAG, "begin %lu bytes -> %s", (unsigned long)image_size,
            partition->label);
        send_ack(message, ESPNOW_OTA_BEGIN_RESPONSE, ESPNOW_OTA_RESULT_OK, 0);
        return true;
    }

    if (message->type == ESPNOW_OTA_DATA_REQUEST) {
        if (!active || !owner_matches(message)) {
            send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
                ESPNOW_OTA_RESULT_NO_SESSION, received_size);
            return true;
        }
        if (message->body_length <= ESPNOW_OTA_DATA_OVERHEAD) {
            send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
                ESPNOW_OTA_RESULT_INVALID, received_size);
            return true;
        }
        uint32_t offset = hid_tunnel_get_u32(message->body);
        size_t chunk_length = message->body_length - ESPNOW_OTA_DATA_OVERHEAD;
        if (offset < received_size) {
            send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
                ESPNOW_OTA_RESULT_OK, received_size);
            return true;
        }
        if (offset != received_size || chunk_length > image_size - received_size) {
            send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
                ESPNOW_OTA_RESULT_OFFSET, received_size);
            return true;
        }
        const uint8_t *chunk = &message->body[ESPNOW_OTA_DATA_OVERHEAD];
        esp_err_t err = esp_ota_write(handle, chunk, chunk_length);
        if (err != ESP_OK || psa_hash_update(&sha256, chunk, chunk_length) != PSA_SUCCESS) {
            ESP_LOGE(TAG, "write at %lu: %s", (unsigned long)offset,
                esp_err_to_name(err));
            reset_session(true);
            send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
                ESPNOW_OTA_RESULT_WRITE_ERROR, offset);
            return true;
        }
        received_size += chunk_length;
        last_activity_us = esp_timer_get_time();
        send_ack(message, ESPNOW_OTA_DATA_RESPONSE,
            ESPNOW_OTA_RESULT_OK, received_size);
        return true;
    }

    if (message->type == ESPNOW_OTA_END_REQUEST) {
        if (!active || !owner_matches(message)) {
            send_ack(message, ESPNOW_OTA_END_RESPONSE,
                ESPNOW_OTA_RESULT_NO_SESSION, received_size);
            return true;
        }
        if (message->body_length != 0 || received_size != image_size) {
            send_ack(message, ESPNOW_OTA_END_RESPONSE,
                ESPNOW_OTA_RESULT_INVALID, received_size);
            return true;
        }
        uint8_t actual_sha256[ESPNOW_OTA_SHA256_SIZE];
        size_t digest_length = 0;
        bool digest_ok = psa_hash_finish(&sha256, actual_sha256,
            sizeof(actual_sha256), &digest_length) == PSA_SUCCESS &&
            digest_length == sizeof(actual_sha256) &&
            memcmp(actual_sha256, expected_sha256, sizeof(actual_sha256)) == 0;
        sha256 = psa_hash_operation_init();
        sha256_initialised = false;
        if (!digest_ok) {
            reset_session(true);
            send_ack(message, ESPNOW_OTA_END_RESPONSE,
                ESPNOW_OTA_RESULT_VERIFY_ERROR, received_size);
            return true;
        }
        const esp_partition_t *completed_partition = partition;
        esp_err_t end_err = esp_ota_end(handle);
        active = false;
        handle = 0;
        if (end_err != ESP_OK ||
            esp_ota_set_boot_partition(completed_partition) != ESP_OK) {
            ESP_LOGE(TAG, "verify/set boot failed: %s", esp_err_to_name(end_err));
            reset_session(false);
            send_ack(message, ESPNOW_OTA_END_RESPONSE,
                ESPNOW_OTA_RESULT_VERIFY_ERROR, received_size);
            return true;
        }
        uint32_t completed_size = received_size;
        reset_session(false);
        send_ack(message, ESPNOW_OTA_END_RESPONSE,
            ESPNOW_OTA_RESULT_OK, completed_size);
        ESP_LOGI(TAG, "verified; restarting into %s", completed_partition->label);
        vTaskDelay(pdMS_TO_TICKS(OTA_RESTART_DELAY_MS));
        esp_restart();
        return true;
    }

    return true;
}

void core_ota_poll(void)
{
    if (active && esp_timer_get_time() - last_activity_us > OTA_SESSION_TIMEOUT_US) {
        ESP_LOGW(TAG, "session timed out at %lu/%lu", (unsigned long)received_size,
            (unsigned long)image_size);
        reset_session(true);
    }
}
