idf_component_register(
  SRCS "zigbee.c"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES nvs_flash esp_timer led
)
