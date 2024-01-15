idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt soc nvs_flash ulp driver lvgl_esp32_drivers 
                                ahrs mpu9250 BH1750 BQ25896 DRV2605 MAX30102
                                pictures)
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_riscv_sources "ulp/ulp_main.c")
set(ulp_exp_dep_srcs "ulp_main.c")
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")
