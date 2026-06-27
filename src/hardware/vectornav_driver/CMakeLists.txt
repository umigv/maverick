cmake_minimum_required(VERSION 3.16)
project(vectornav_driver)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GeographicLib REQUIRED)

add_subdirectory(third_party/vectornav_sdk/src vectornav_sdk)
# Enable all GNSS fields in CompositeData; the default bitmask omits GNSS_GNSS1SATINFO_BIT which is needed to populate
# NavSatFix service flags
target_compile_definitions(oVnSensor PUBLIC
  GNSS_GROUP_ENABLE=0xFFFFFFFF
  GNSS2_GROUP_ENABLE=0xFFFFFFFF
)

add_executable(${PROJECT_NAME}
  src/vectornav_driver.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  third_party/vectornav_sdk/include
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  oVnSensor
  GeographicLib::GeographicLib
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
