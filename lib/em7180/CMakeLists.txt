set(EM7180_SRCS_COMMON
    config.c
    data.c
    debug.c
    info.c
    init.c
    param.c
    state.c
)
set(EM7180_INCS_COMMON
    include
)
set(EM7180_LIBS_COMMON
    crosslog
    crossi2c
    usfs
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(em7180)
zephyr_library_sources(
    ${EM7180_SRCS_COMMON}
)
target_include_directories(em7180 PUBLIC
    ${EM7180_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${EM7180_LIBS_COMMON}
)
elseif(COMMAND idf_component_register)
set(COMPONENT_SRCS
    ${EM7180_SRCS_COMMON}
)
set(COMPONENT_ADD_INCLUDEDIRS
    ${EM7180_INCS_COMMON}
)
set(COMPONENT_PRIV_REQUIRES
    ${EM7180_LIBS_COMMON}
)
register_component()
else()
cmake_minimum_required(VERSION 2.8)
project(em7180)

add_library(em7180
    ${EM7180_SRCS_COMMON}
)
target_include_directories(em7180 PUBLIC
    ${EM7180_INCS_COMMON}
)
target_link_libraries(em7180
    ${EM7180_LIBS_COMMON}
)
endif()
