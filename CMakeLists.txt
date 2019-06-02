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
else()
cmake_minimum_required(VERSION 2.8)
project(usfs)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS ON)

add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Wstrict-prototypes
    -Wmissing-prototypes
    -Werror
)

add_library(usfs SHARED
    ${USFS_SRCS_COMMON}
)
target_include_directories(usfs PUBLIC
    ${USFS_INCS_COMMON}
)
target_link_libraries(usfs
    ${USFS_LIBS_COMMON}
)

add_subdirectory(tools)
endif()

add_subdirectory(lib/em7180)
add_subdirectory(lib/m24512)
