set(EMPL_SRCS_COMMON
    inv_mpu.c
)
set(EMPL_INCS_COMMON
    include
)
set(EMPL_LIBS_COMMON
    crosslog
    crossi2c
    m
)
set(EMPL_FLAGS_COMMON
    -DMPU9250
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
)
target_compile_options(eMPL PRIVATE
    ${EMPL_FLAGS_COMMON}
)
endif()