idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES icm42688 net_send oled battery power_mgmt esp_hw_support driver esp_timer)
