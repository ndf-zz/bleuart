idf_component_register(SRCS "main.c"
	REQUIRES bt nvs_flash esp_driver_uart esp_pm esp_app_format esp_wifi esp_https_ota esp_netif mbedtls
	INCLUDE_DIRS ".")
