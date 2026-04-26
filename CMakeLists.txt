cmake_minimum_required(VERSION 3.16)
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

include(CTest)
if (BUILD_TESTING)
    set(TEST_SRC_FILES ${SRC_FILES})
    list(REMOVE_ITEM TEST_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

    add_executable(smoke_test tests/smoke_test.cpp ${TEST_SRC_FILES})
    target_include_directories(smoke_test PRIVATE src)
    add_test(NAME smoke_test COMMAND smoke_test)
endif()
