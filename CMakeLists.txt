cmake_minimum_required(VERSION 3.8)
project(adhoc_wemos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_executable(pubsub_example src/pubsub_example.cpp)
ament_target_dependencies(pubsub_example rclcpp std_msgs sensor_msgs) 

add_executable(pubsub_wemos src/pubsub_wemos.cpp)
target_link_libraries(pubsub_wemos ${Boost_LIBRARIES})
ament_target_dependencies(pubsub_wemos rclcpp std_msgs sensor_msgs) 

add_executable(strsub_wemos src/wemos_strsub.cpp)
target_link_libraries(strsub_wemos ${Boost_LIBRARIES})
ament_target_dependencies(strsub_wemos rclcpp std_msgs sensor_msgs)

install(TARGETS
  pubsub_example
  pubsub_wemos
  strsub_wemos
  DESTINATION lib/${PROJECT_NAME})
  

ament_package()
