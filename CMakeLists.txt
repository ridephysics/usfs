set(USFS_SRCS_COMMON
    usfs.c
)
set(USFS_INCS_COMMON
    include
)
set(USFS_LIBS_COMMON
    crossi2c
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(usfs)
zephyr_library_sources(
    ${USFS_SRCS_COMMON}
)
target_include_directories(usfs PUBLIC
    ${USFS_INCS_COMMON}
)
zephyr_library_link_libraries(
    ${USFS_LIBS_COMMON}
)
elseif(COMMAND idf_component_register)
set(COMPONENT_SRCS
    ${USFS_SRCS_COMMON}
)
set(COMPONENT_ADD_INCLUDEDIRS
    ${USFS_INCS_COMMON}
)
set(COMPONENT_PRIV_REQUIRES
    ${USFS_LIBS_COMMON}
)
register_component()
else()
cmake_minimum_required(VERSION 2.8)
project(usfs)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS ON)

add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Werror
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-prototypes -Wmissing-prototypes")

add_library(usfs SHARED
    ${USFS_SRCS_COMMON}
)
target_include_directories(usfs PUBLIC
    ${USFS_INCS_COMMON}
)
target_link_libraries(usfs
    ${USFS_LIBS_COMMON}
)

add_subdirectory(${COMPONENTS_DIR}/crosslog crosslog)
add_subdirectory(${COMPONENTS_DIR}/crossi2c crossi2c)

add_subdirectory(tools)
endif()

if(NOT COMMAND idf_component_register)
add_subdirectory(lib/em7180)
add_subdirectory(lib/m24512)
add_subdirectory(lib/bmp280)
add_subdirectory(lib/eMPL)
endif()
