cmake_minimum_required(VERSION 3.16)
project(template_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} src/template_cpp.cpp)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
