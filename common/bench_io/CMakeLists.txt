idf_component_register(SRCS "bench_io.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_driver_gpio esp_driver_usb_serial_jtag esp_timer
                                esp_system esp_hw_support esp_app_format)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
