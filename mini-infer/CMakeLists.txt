cmake_minimum_required(VERSION 3.16)
project(mini_infer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library(mi_naive
  src/naive/matmul.cpp
  src/naive/conv2d.cpp
)
target_include_directories(mi_naive PUBLIC src)

add_executable(test_correctness tests/test_correctness.cpp)
target_link_libraries(test_correctness PRIVATE mi_naive)

add_executable(bench_naive bench/bench_main.cpp)
target_link_libraries(bench_naive PRIVATE mi_naive)
target_include_directories(bench_naive PRIVATE bench)
target_compile_definitions(bench_naive PRIVATE
  MI_BUILD_FLAGS="${CMAKE_CXX_FLAGS_RELEASE}")

enable_testing()
add_test(NAME correctness
         COMMAND test_correctness ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)
