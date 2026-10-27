FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc dht11 i2c_lcd)