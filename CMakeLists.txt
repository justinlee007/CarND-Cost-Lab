cmake_minimum_required(VERSION 3.7)
project(CarND_Cost_Lab)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp src/cost.cpp src/cost.h src/road.cpp src/road.h src/vehicle.cpp src/vehicle.h)
add_executable(CarND_Cost_Lab ${SOURCE_FILES})