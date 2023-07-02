set(BSP_COMMON_DIR ${CMAKE_SOURCE_DIR}/bsp/bsp_common)
set(TARGET_REQUIRED_LIBS tflite)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tflite_camera_psram.ld)
set(TARGET_REQUIRED_PRIVATE_INCLUDE ${BSP_COMMON_DIR}/psram ${BSP_COMMON_DIR}/image_sensor ${CMAKE_CURRENT_LIST_DIR}/wifi_at_http)
file(GLOB_RECURSE ref
"${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
)
set(TARGET_REQUIRED_SRCS ${BSP_COMMON_DIR}/psram/bsp_sf_psram.c ${BSP_COMMON_DIR}/image_sensor/bsp_image_sensor.c ${CMAKE_CURRENT_LIST_DIR}/wifi_at_http/at.c ${ref})
set(mains main.c)
generate_bin()


