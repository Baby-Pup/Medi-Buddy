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

# 1. 라이브러리 이름은 명확하게
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

# 5. pluginlib.xml 설치 (내 패키지 share 폴더에만!)
install(FILES pluginlib.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
