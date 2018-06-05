cmake_minimum_required(VERSION 3.10)
project(untangler)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp OBJ_Loader.h)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(untangler ${SOURCE_FILES})

target_link_libraries(untangler ${CERES_LIBRARIES})