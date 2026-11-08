idf_component_register(SRCS "main.c" "oled.c"
                       INCLUDE_DIRS "."
                       REQUIRES transfer_core bench_io esp_driver_i2c esp_system)
