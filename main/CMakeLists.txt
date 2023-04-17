idf_component_register(SRCS "operations.c" "hid_touch_gestures.c" "mpu6500.c" "ble_go_main.c" "paj7620.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)


