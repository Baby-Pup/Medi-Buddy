cmake_minimum_required(VERSION 3.8)
project(ai_inference_cpp)

# 기본 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 실행파일 생성
add_executable(heatmap_bias_node
  src/heatmap_bias_node.cpp
)

# 종속성 연결
ament_target_dependencies(heatmap_bias_node
  rclcpp
  std_msgs
)

# 설치 규칙
install(TARGETS
  heatmap_bias_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
