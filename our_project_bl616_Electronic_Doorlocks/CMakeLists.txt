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

target_sources(app PRIVATE lcd/lcd.c)
target_sources(app PRIVATE lcd/font.c)
target_sources(app PRIVATE lcd/spi/ili9341_spi.c)
target_sources(app PRIVATE lcd/spi/bl_spi_hard_4.c)

target_sources(app PRIVATE lwip_http/lwip_http.c )
target_sources(app PRIVATE sev_btble/btble.c ) 

sdk_add_include_directories(finger_identify)
target_sources(app PRIVATE  finger_identify/as608.c)
target_sources(app PRIVATE  finger_identify/finger_identify.c)

sdk_add_include_directories(keyboard)
target_sources(app PRIVATE  keyboard/keyboard.c)

sdk_add_include_directories(.)

sdk_set_main_file(main.c)

project(mqtt)
