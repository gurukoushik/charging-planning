cmake_minimum_required(VERSION 3.10)

project(charging-planning)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(ortools CONFIG REQUIRED)

# Add your source files and set up the build
add_executable(plan main.cpp network.cpp)
target_link_libraries(plan ortools::ortools)
