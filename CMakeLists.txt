cmake_minimum_required(VERSION 3.10)

project(charging-planning)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ortools CONFIG REQUIRED)

file(GLOB SRC
     "./*.cpp"
)

# Add your source files and set up the build
add_executable(plan main.cpp ${SRC})
target_link_libraries(plan ortools::ortools)
