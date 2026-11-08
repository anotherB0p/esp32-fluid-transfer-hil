idf_component_register(SRCS "main.c" "status_led.c" "fault_log.c" "ota_console.c"
                       INCLUDE_DIRS "."
                       REQUIRES transfer_core bench_io esp_driver_rmt esp_system nvs_flash
                                app_update mbedtls esp_app_format)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# idf.py ... -DOTA_TEST_BAD_IMAGE=1 build -> an image whose post-update self-test always
# fails, used on the bench to prove that the bootloader rolls back.
if(OTA_TEST_BAD_IMAGE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OTA_TEST_BAD_IMAGE=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OTA_TEST_BAD_IMAGE=0)
endif()
