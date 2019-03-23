cmake_minimum_required(VERSION 2.8)
project(usfs)

add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Wstrict-prototypes
    -Werror
)

add_library(usfs SHARED
    usfs.c
)
target_include_directories(usfs PUBLIC
    include
)
target_link_libraries(usfs
    crossi2c
)

add_subdirectory(lib/crosslog)
add_subdirectory(lib/crossi2c)
add_subdirectory(lib/em7180)
add_subdirectory(lib/m24512)
add_subdirectory(tools)
