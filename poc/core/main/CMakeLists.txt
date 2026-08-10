idf_component_register(
    SRCS
        "hello_world_main.c"
        "espnow_echo.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        usb
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        hid_tunnel_protocol
)
