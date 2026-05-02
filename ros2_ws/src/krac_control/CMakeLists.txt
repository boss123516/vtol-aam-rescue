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
# [추가] GPS 메시지 사용을 위해 필요
find_package(sensor_msgs REQUIRED) 

include_directories(include)

# --------------------------------------------------------
# 1.원 궤도 알고리즘 노드 
# --------------------------------------------------------
add_executable(vtol_fsm src/vtol_fsm.cpp)
ament_target_dependencies(vtol_fsm
  rclcpp
  mavros_msgs
  geometry_msgs
  krac_interfaces
  tf2 
  tf2_geometry_msgs
  sensor_msgs # [추가]
  std_srvs
)

# --------------------------------------------------------
# 2.P-Turn 방식 
# --------------------------------------------------------
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

# --------------------------------------------------------
# 3. 설치 (Install) 설정
# --------------------------------------------------------
install(TARGETS 
  vtol_fsm 
  vtol_fsm_P  # [추가] 여기에도 꼭 적어줘야 install 폴더에 생성됩니다!
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/mission_loader.py  # <-- 여기 경로가 중요합니다
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
