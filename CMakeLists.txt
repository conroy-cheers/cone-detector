cmake_minimum_required(VERSION 3.5)
project(cone_detector)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


add_executable(cone_detector cone_detector.cpp)
ament_target_dependencies(cone_detector rclcpp std_msgs sensor_msgs OpenCV cv_bridge image_transport)

install(TARGETS
        cone_detector
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
