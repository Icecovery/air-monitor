idf_component_register(
  SRCS 
    "main.c"
    "led/led.c"
    "led/led_controller.c"
    "tvoc/tvoc.c"
    "epaper/epaper.c"
    "epaper/EPD_2in15b.c"
    "epaper/Device_Config.c"
    "epaper/GUI_Paint.c"
    "epaper/fonts/font8.c"
    "epaper/fonts/font12.c"
    "epaper/fonts/font16.c"
    "epaper/fonts/font20.c"
    "epaper/fonts/font24.c"
  INCLUDE_DIRS "."
)
