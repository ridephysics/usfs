set(M24512_SRCS_COMMON
    m24512.c
)
set(M24512_INCS_COMMON
    include
)
set(M24512_LIBS_COMMON
    crosslog
    crossi2c
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(m24512)
zephyr_library_sources(
    ${M24512_SRCS_COMMON}
)
target_include_directories(m24512 PUBLIC
    ${M24512_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${M24512_LIBS_COMMON}
)
elseif(COMMAND idf_component_register)
set(COMPONENT_SRCS
    ${M24512_SRCS_COMMON}
)
set(COMPONENT_ADD_INCLUDEDIRS
    ${M24512_INCS_COMMON}
)
set(COMPONENT_PRIV_REQUIRES
    ${M24512_LIBS_COMMON}
)
register_component()
else()
cmake_minimum_required(VERSION 2.8)
project(m24512)

add_library(m24512
    ${M24512_SRCS_COMMON}
)
target_include_directories(m24512 PUBLIC
    ${M24512_INCS_COMMON}
)
target_link_libraries(m24512
    ${M24512_LIBS_COMMON}
)
endif()
