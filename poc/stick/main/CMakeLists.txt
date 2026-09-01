idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_app_format
        hid_tunnel_protocol
        esp_driver_usb_serial_jtag
        m5stack__m5gfx
)
