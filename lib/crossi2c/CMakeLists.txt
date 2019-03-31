set(CROSSI2C_INCS_COMMON
    include
)
set(CROSSI2C_LIBS_COMMON
    crosslog
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(crossi2c)
zephyr_library_sources(
    zephyr.c
)
target_include_directories(crossi2c PUBLIC
    ${CROSSI2C_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${CROSSI2C_LIBS_COMMON}
)
else()
cmake_minimum_required(VERSION 2.8)
project(crossi2c)

add_library(crossi2c
    linux.c
)
target_include_directories(crossi2c PUBLIC
    ${CROSSI2C_INCS_COMMON}
)
target_link_libraries(crossi2c
    ${CROSSI2C_LIBS_COMMON}
)
endif()
