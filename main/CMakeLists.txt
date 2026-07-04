idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES led tvoc epaper zigbee esp_driver_gpio
)
