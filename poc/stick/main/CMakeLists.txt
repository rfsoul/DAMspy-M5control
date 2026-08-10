idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        hid_tunnel_protocol
)
