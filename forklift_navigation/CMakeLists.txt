cmake_minimum_required(VERSION 3.8)
project(forklift_navigation)

# 默认使用 C++17 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 安装 Python 模块（forklift_navigation/ 目录作为 Python 包）
ament_python_install_package(${PROJECT_NAME})

# 安装 Python 可执行脚本
install(PROGRAMS
  forklift_navigation/navigate_to_point.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件目录
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装地图文件目录
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# 安装 launch 文件目录
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
