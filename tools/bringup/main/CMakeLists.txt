idf_component_register(SRCS "bringup.c"
                       REQUIRES esp_driver_rmt esp_driver_i2c esp_driver_uart esp_driver_gpio esp_hw_support)

# idf.py ... -DBRINGUP_PINMAP=1 build  -> wiring diagnostic instead of the link test
if(BRINGUP_PINMAP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BRINGUP_PINMAP=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BRINGUP_PINMAP=0)
endif()
