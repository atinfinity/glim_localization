cmake_minimum_required(VERSION 3.16)
project(check_deps LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gtsam_points REQUIRED)
find_package(glim REQUIRED)

add_executable(check_deps main.cpp)
target_link_libraries(check_deps PRIVATE glim::glim gtsam_points::gtsam_points ${CMAKE_DL_LIBS})
