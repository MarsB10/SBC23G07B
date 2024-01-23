# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
set(requires "")
idf_build_get_property(target IDF_TARGET)
idf_component_get_property(nvs_flash nvs_flash COMPONENT_LIB)


if(${target} STREQUAL "linux")
    list(APPEND requires esp_stubs esp_event esp-tls esp_http_client protocol_examples_common nvs_flash)
endif()
idf_component_register(SRCS "http.c" "connect.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires}
                    EMBED_TXTFILES howsmyssl_com_root_cert.pem
                    postman_root_cert.pem
                    PRIV_REQUIRES esp_event
                    PRIV_REQUIRES esp_http_client
                    PRIV_REQUIRES esp-tls
                    PRIV_REQUIRES esp_netif
                    PRIV_REQUIRES nvs_flash)