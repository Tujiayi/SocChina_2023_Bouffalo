cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

# Define
sdk_add_compile_definitions(-DCONFIG_CLI_CMD_ENABLE)
#sdk_add_compile_definitions(-DWIFI_DEMO_TEST)
#sdk_add_compile_definitions(-DBL616_DHCP_DEBUG)
sdk_add_include_directories(sev_btble)
sdk_add_include_directories(lwip_http)
sdk_add_include_directories(lcd)
sdk_add_include_directories(${PROJECT_SOURCE_DIR}/spi)
sdk_add_include_directories(image)

target_sources(app PRIVATE lwip_http/lwip_http.c )
target_sources(app PRIVATE sev_btble/btble.c )
target_sources(app PRIVATE image/image_sensor.c)
target_sources(app PRIVATE image/image_trans.c)
target_sources(app PRIVATE lcd/lcd.c)
target_sources(app PRIVATE lcd/font.c)
target_sources(app PRIVATE lcd/spi/st7789v_spi.c)
target_sources(app PRIVATE lcd/spi/bl_spi_hard_4.c)

sdk_add_include_directories(es8388)
target_sources(app PRIVATE  es8388/bsp_es8388.c)  

sdk_add_include_directories(jpeg)
target_sources(app PRIVATE  jpeg/jpeg_head.c)

sdk_add_include_directories(.)

sdk_set_main_file(main.c)

project(mqtt)
