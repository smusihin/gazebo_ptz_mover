cmake_minimum_required(VERSION 3.5)

project(gazebo_ptz_mover LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)    
    find_package(trajectory_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
endif()


add_executable(${PROJECT_NAME} main.cpp)

if (DEFINED ENV{AMENT_PREFIX_PATH})
    ament_target_dependencies(${PROJECT_NAME}
      "rclcpp"
      "sensor_msgs"
      "trajectory_msgs"
      )
endif()

