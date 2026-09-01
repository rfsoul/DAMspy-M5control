idf_component_register(
    SRCS "radio_diag.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif nvs_flash
)
