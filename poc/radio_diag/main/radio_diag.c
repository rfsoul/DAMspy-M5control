#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define RADIO_CHANNEL 6

static const char *TAG = "radio_diag";
static const uint8_t broadcast[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint8_t sender[ESP_NOW_ETH_ALEN];
} __attribute__((packed)) diagnostic_packet_t;

static void send_callback(
    const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "TX callback dst=" MACSTR " status=%s",
        MAC2STR(info->des_addr),
        status == ESP_NOW_SEND_SUCCESS ? "ok" : "fail");
}

static void receive_callback(
    const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    uint32_t sequence = 0;
    if (length == sizeof(diagnostic_packet_t)) {
        diagnostic_packet_t packet;
        memcpy(&packet, data, sizeof(packet));
        sequence = packet.sequence;
    }
    ESP_LOGI(TAG, "RX src=" MACSTR " len=%d seq=%" PRIu32 " rssi=%d",
        MAC2STR(info->src_addr), length, sequence,
        info->rx_ctrl != NULL ? info->rx_ctrl->rssi : 0);
}

static void initialise_radio(uint8_t station_mac[ESP_NOW_ETH_ALEN])
{
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
            WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, station_mac));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_callback));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = RADIO_CHANNEL;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_11B,
        .rate = WIFI_PHY_RATE_1M_L,
        .ersu = false,
        .dcm = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(broadcast, &rate));
}

void app_main(void)
{
    uint8_t station_mac[ESP_NOW_ETH_ALEN];
    initialise_radio(station_mac);
    ESP_LOGI(TAG, "READY channel=%d rate=1Mbps mac=" MACSTR,
        RADIO_CHANNEL, MAC2STR(station_mac));

    diagnostic_packet_t packet = {
        .magic = 0x52464447,
        .sequence = 0,
    };
    memcpy(packet.sender, station_mac, ESP_NOW_ETH_ALEN);

    while (true) {
        packet.sequence++;
        esp_err_t error = esp_now_send(
            broadcast, (const uint8_t *)&packet, sizeof(packet));
        ESP_LOGI(TAG, "TX queue seq=%" PRIu32 " result=%s",
            packet.sequence, esp_err_to_name(error));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
