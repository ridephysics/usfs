cmake_minimum_required(VERSION 2.8)
project(usfstools)

add_executable(usfs_test
    test.c
)

target_link_libraries(usfs_test
    usfs
    crosslog
    crossi2c
    em7180
)
