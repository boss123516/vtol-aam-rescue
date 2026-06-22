cmake_minimum_required(VERSION 3.8)
project(krac_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(krac_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(vtol_fsm src/vtol_fsm.cpp)
ament_target_dependencies(vtol_fsm
  rclcpp
  mavros_msgs
  geometry_msgs
  krac_interfaces
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
)

add_executable(vtol_fsm_P src/vtol_fsm_P.cpp)
ament_target_dependencies(vtol_fsm_P
  rclcpp
  mavros_msgs
  geometry_msgs
  krac_interfaces
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS
  vtol_fsm
  vtol_fsm_P
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/mission_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/way3.plan
  DESTINATION share/${PROJECT_NAME}/mission
)

ament_package()
