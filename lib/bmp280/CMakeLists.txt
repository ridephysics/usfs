set(BMP280_SRCS_COMMON
    ${BMP280_DIR}/bmp280.c
    bmp280.c
)
set(BMP280_INCS_COMMON
    include
    ${BMP280_DIR}
)
set(BMP280_LIBS_COMMON
    crosslog
    crossi2c
    usfs
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(bmp280)
zephyr_library_sources(
    ${BMP280_SRCS_COMMON}
)
target_include_directories(bmp280 PUBLIC
    ${BMP280_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${BMP280_LIBS_COMMON}
)
elseif(COMMAND idf_component_register)
set(COMPONENT_SRCS
    ${BMP280_SRCS_COMMON}
)
set(COMPONENT_ADD_INCLUDEDIRS
    ${BMP280_INCS_COMMON}
)
set(COMPONENT_PRIV_REQUIRES
    ${BMP280_LIBS_COMMON}
)
register_component()
else()
cmake_minimum_required(VERSION 2.8)
project(bmp280)

add_library(bmp280
    ${BMP280_SRCS_COMMON}
)
target_include_directories(bmp280 PUBLIC
    ${BMP280_INCS_COMMON}
)
target_link_libraries(bmp280
    ${BMP280_LIBS_COMMON}
)
endif()
