idf_component_register(
    SRCS
        "src/power_mgmt.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_timer
        esp_hw_support
        ulp
        log
        freertos
)

# ULP RISC-V 程序: 编译并内嵌到主核 Flash
ulp_embed_binary(power_mgmt_ulp "ulp/ulp_main.c" "")
