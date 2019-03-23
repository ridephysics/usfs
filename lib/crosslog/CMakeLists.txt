cmake_minimum_required(VERSION 2.8)
project(crosslog)

add_library(crosslog
    crosslog.c
)
target_include_directories(crosslog PUBLIC
    include
)
