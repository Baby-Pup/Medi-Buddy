cmake_minimum_required(VERSION 3.5)
project(dwb_omega_critic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dwb_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# 1. 라이브러리 생성
add_library(dwb_omega_critic SHARED
  src/omega_direction_critic.cpp
)

# 2. 의존성 연결
ament_target_dependencies(dwb_omega_critic
  rclcpp
  dwb_core
  pluginlib
  nav_2d_msgs
  geometry_msgs
  std_msgs
)

# 3. 라이브러리 설치
install(TARGETS dwb_omega_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 4. 헤더 설치
install(DIRECTORY include/
  DESTINATION include
)

# 5. 🟢 pluginlib + ament index 등록
# (pluginlib 공식 매크로 — 이 줄이 모든 걸 자동 해결함)
pluginlib_export_plugin_description_file(dwb_core pluginlib.xml)

ament_package()
