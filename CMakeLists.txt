cmake_minimum_required(VERSION 3.8)
project(gzsatellite)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(CURL REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

## Declare a C++ library
ament_auto_add_library(TilePlugin SHARED src/TilePlugin.cpp src/tileloader.cpp src/modelcreator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(TilePlugin ${GAZEBO_LIBRARIES}  ${OpenCV_LIBS})

ament_auto_package(INSTALL_TO_SHARE
        worlds
        launch)
