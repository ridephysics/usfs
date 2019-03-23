cmake_minimum_required(VERSION 2.8)
project(m24512)

add_library(m24512
    m24512.c
)
target_include_directories(m24512 PUBLIC
    include
)
target_link_libraries(m24512
    crosslog
    crossi2c
)
