idf_component_register(SRCS "main.c" "status_led.c"
                       INCLUDE_DIRS "."
                       REQUIRES transfer_core bench_io esp_driver_rmt esp_system)
