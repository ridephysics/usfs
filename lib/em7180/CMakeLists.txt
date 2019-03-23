cmake_minimum_required(VERSION 2.8)
project(em7180)

add_library(em7180
    config.c
    data.c
    debug.c
    info.c
    init.c
    param.c
    state.c
)
target_include_directories(em7180 PUBLIC
    include
)
target_link_libraries(em7180
    crosslog
    crossi2c
)
