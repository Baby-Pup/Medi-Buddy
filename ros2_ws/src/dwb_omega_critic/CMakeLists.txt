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
add_library(dwb_omega_critic SHARED src/omega_direction_critic.cpp)

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

# 4. 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include
)

# 5. XML 파일 설치 (이거 다시 살립니다!)
install(FILES pluginlib.xml
  DESTINATION share/${PROJECT_NAME}
)

# 6. ⭐ [핵심] Ament Index 강제 등록 ⭐
# (매크로가 안 되니까 수동으로 빈 파일을 만들어서 꽂아버립니다)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/dwb_omega_critic "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dwb_omega_critic
  DESTINATION share/ament_index/resource_index/dwb_core
)

ament_package()