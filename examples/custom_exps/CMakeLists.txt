cmake_minimum_required(VERSION 2.8.12)
project(manifCustom)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
add_executable(${PROJECT_NAME} src/sam.cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
# Find the manif library
find_package(manif REQUIRED)
# Add manif include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
