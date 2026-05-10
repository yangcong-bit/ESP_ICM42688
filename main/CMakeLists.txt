idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES icm42688 net_send oled driver esp_timer)
