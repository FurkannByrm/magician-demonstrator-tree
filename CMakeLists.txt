cmake_minimum_required(VERSION 3.8)
project(magician_demonstrator_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(dsr_msgs2 REQUIRED)

# set(BEHAVIOR_SOURCES
# src/demonstrator_bahavior.cpp 
# )

set(TARGET_DEPENDS
    rclcpp
    behaviortree_cpp
    sensor_msgs
    std_srvs
    dsr_msgs2
)

include_directories(include)
add_executable(behavior_node src/main.cpp
src/demonstrator_bahavior.cpp
)
ament_target_dependencies(behavior_node ${TARGET_DEPENDS})

install(TARGETS
behavior_node
DESTINATION lib/${PROJECT_NAME})

ament_package()
