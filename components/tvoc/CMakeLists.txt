idf_component_register(
  SRCS "tvoc.c"
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_gpio
  PRIV_REQUIRES esp_driver_uart
)
