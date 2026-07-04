idf_component_register(
  SRCS
    "epaper.c"
    "EPD_2in15g.c"
    "GUI_Paint.c"
    "Device_Config.c"
    "fonts/font8.c"
    "fonts/font12.c"
    "fonts/font16.c"
    "fonts/font20.c"
    "fonts/font24.c"
    "fonts/rec_mono_36.c"
    "fonts/rec_mono_40.c"
  INCLUDE_DIRS "include"
  PRIV_INCLUDE_DIRS "." "fonts"
  PRIV_REQUIRES esp_driver_gpio
)
