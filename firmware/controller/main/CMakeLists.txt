idf_component_register(SRCS "main.c" "status_led.c" "fault_log.c"
                       INCLUDE_DIRS "."
                       REQUIRES transfer_core bench_io esp_driver_rmt esp_system nvs_flash)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
