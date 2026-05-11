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

file(GLOB_RECURSE CORE_SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
)
list(FILTER CORE_SRC_FILES EXCLUDE REGEX ".*/main.cpp$")

enable_testing()
add_executable(phase2_tests tests/phase2_tests.cpp ${CORE_SRC_FILES})
target_include_directories(phase2_tests PRIVATE src)
add_test(NAME phase2_tests COMMAND phase2_tests)

add_executable(cbs_solver_tests tests/cbs_solver_tests.cpp ${CORE_SRC_FILES})
target_include_directories(cbs_solver_tests PRIVATE src)
add_test(NAME cbs_solver_tests COMMAND cbs_solver_tests)
