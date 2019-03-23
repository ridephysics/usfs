cmake_minimum_required(VERSION 2.8)
project(crossi2c)

add_library(crossi2c
    linux.c
)
target_include_directories(crossi2c PUBLIC
    include
)
target_link_libraries(crossi2c
    crosslog
)
