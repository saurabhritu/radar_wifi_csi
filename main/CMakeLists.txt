idf_component_register(
    SRCS
        "main.c"
        "wifi_csi.c"
        "http_server.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        json
        log
)

# Link the math library for sqrtf() in wifi_csi.c
target_link_libraries(${COMPONENT_LIB} INTERFACE m)
