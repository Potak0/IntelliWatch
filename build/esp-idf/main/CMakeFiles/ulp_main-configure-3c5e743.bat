@echo off
cd /D D:\ESP_Project\IntelliWatch\build\esp-idf\main\ulp_main || (set FAIL_LINE=2& goto :ABORT)
D:\Espressif\442\tools\cmake\3.23.1\bin\cmake.exe -DCMAKE_GENERATOR=Ninja -DCMAKE_TOOLCHAIN_FILE=D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/ulp/cmake/toolchain-ulp-riscv.cmake -DULP_S_SOURCES=D:/ESP_Project/IntelliWatch/main/ulp/ulp_main.c -DULP_APP_NAME=ulp_main -DCOMPONENT_DIR=D:/ESP_Project/IntelliWatch/main -DCOMPONENT_INCLUDES=D:/ESP_Project/IntelliWatch/main;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/newlib/platform_include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/freertos/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/freertos/include/esp_additions/freertos;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/freertos/port/xtensa/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/freertos/include/esp_additions;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hw_support/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hw_support/include/soc;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hw_support/include/soc/esp32s3;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hw_support/port/esp32s3/.;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hw_support/port/esp32s3/private_include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/heap/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/log/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/lwip/include/apps;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/lwip/include/apps/sntp;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/lwip/lwip/src/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/lwip/port/esp32/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/lwip/port/esp32/include/arch;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/soc/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/soc/esp32s3/.;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/soc/esp32s3/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/hal/esp32s3/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/hal/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/hal/platform_port/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_rom/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_rom/include/esp32s3;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_rom/esp32s3;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_common/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_system/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_system/port/soc;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_system/port/public_compat;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/xtensa/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/xtensa/esp32s3/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/driver/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/driver/esp32s3/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_pm/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_ringbuf/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/efuse/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/efuse/esp32s3/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/vfs/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_wifi/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_event/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_netif/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_eth/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/tcpip_adapter/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_phy/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_phy/esp32s3/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_ipc/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/app_trace/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_timer/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/common/osi/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/include/esp32s3/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/common/api/include/api;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/common/btc/profile/esp/blufi/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/common/btc/profile/esp/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bt/host/bluedroid/api/include/api;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/nvs_flash/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/spi_flash/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/port/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/esp_crt_bundle/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/ulp/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/lvgl_esp32_drivers;D:/ESP_Project/IntelliWatch/components/lvgl_esp32_drivers/lvgl_tft;D:/ESP_Project/IntelliWatch/components/lvgl_esp32_drivers/lvgl_touch;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/managed_components/lvgl__lvgl;D:/ESP_Project/IntelliWatch/managed_components/lvgl__lvgl/src;D:/ESP_Project/IntelliWatch/managed_components;D:/ESP_Project/IntelliWatch/managed_components/lvgl__lvgl/examples;D:/ESP_Project/IntelliWatch/managed_components/lvgl__lvgl/demos;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/ahrs/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/mpu9250/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/BH1750/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/BQ25896/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/DRV2605/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/MAX30102/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/components/pictures/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/app_update/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/bootloader_support/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/asio/asio/asio/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/asio/port/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/cbor/port/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/cmock/CMock/src;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/unity/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/unity/unity/src;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/coap/port/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/coap/port/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/coap/libcoap/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/console;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp-tls;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp-tls/esp-tls-crypto;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_adc_cal/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_gdbstub/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_gdbstub/xtensa;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_gdbstub/esp32s3;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_hid/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_http_client/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/nghttp/port/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/nghttp/nghttp2/lib/includes;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_http_server/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_https_ota/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_lcd/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_lcd/interface;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_local_ctrl/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/protocomm/include/common;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/protocomm/include/security;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/protocomm/include/transports;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_serial_slave_link/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/sdmmc/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/esp_websocket_client/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/tcp_transport/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/espcoredump/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/espcoredump/include/port/xtensa;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/expat/expat/expat/lib;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/expat/port/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/fatfs/diskio;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/fatfs/vfs;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/fatfs/src;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/wear_levelling/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/freemodbus/common/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/idf_test/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/idf_test/include/esp32s3;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/ieee802154/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/jsmn/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/json/cJSON;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/libsodium/libsodium/src/libsodium/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/libsodium/port_include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mdns/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/mqtt/esp-mqtt/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/openssl/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/perfmon/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/protobuf-c/protobuf-c;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/pthread/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/spiffs/include;D:/ESP_Project/IntelliWatch/build/config;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/usb/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/wifi_provisioning/include;D:/ESP_Project/IntelliWatch/build/config;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/wpa_supplicant/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/wpa_supplicant/port/include;D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/wpa_supplicant/esp_supplicant/include;D:/ESP_Project/IntelliWatch/build/config -DIDF_PATH=D:/Espressif/442/frameworks/esp-idf-v4.4.2 -DIDF_TARGET=esp32s3 -DSDKCONFIG_HEADER=D:/ESP_Project/IntelliWatch/build/config/sdkconfig.h -DPYTHON=D:/Espressif/442/python_env/idf4.4_py3.8_env/Scripts/python.exe -DULP_COCPU_IS_RISCV=ON -GNinja D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/ulp/cmake || (set FAIL_LINE=3& goto :ABORT)
D:\Espressif\442\tools\cmake\3.23.1\bin\cmake.exe -E touch D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp/ulp_main-configure || (set FAIL_LINE=4& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%