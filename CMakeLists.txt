cmake_minimum_required(VERSION 20)
project(MAPF-Competition LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
)

add_executable(searchclient ${SRC_FILES})

target_include_directories(searchclient PRIVATE src)