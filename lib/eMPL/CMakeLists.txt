set(EMPL_SRCS_COMMON
    bias.c
    compass.c
    dmp.c
    fifo.c
    interrupt.c
    inv_mpu.c
    selftest.c
)
set(EMPL_INCS_COMMON
    include
)
set(EMPL_LIBS_COMMON
    crosslog
    crossi2c
    usfs
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(eMPL)
zephyr_library_sources(
    ${EMPL_SRCS_COMMON}
)
target_include_directories(eMPL PUBLIC
    ${EMPL_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${EMPL_LIBS_COMMON}
)
elseif(COMMAND idf_component_register)
set(COMPONENT_SRCS
    ${EMPL_SRCS_COMMON}
)
set(COMPONENT_ADD_INCLUDEDIRS
    ${EMPL_INCS_COMMON}
)
set(COMPONENT_PRIV_REQUIRES
    ${EMPL_LIBS_COMMON}
)
register_component()
else()
cmake_minimum_required(VERSION 2.8)
project(eMPL)

add_library(eMPL
    ${EMPL_SRCS_COMMON}
)
target_include_directories(eMPL PUBLIC
    ${EMPL_INCS_COMMON}
)
target_link_libraries(eMPL
    ${EMPL_LIBS_COMMON}
    m
)
endif()
