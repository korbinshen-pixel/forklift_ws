cmake_minimum_required(VERSION 3.8)
project(forklift_gazebo)

# 查找ament_cmake依赖
find_package(ament_cmake REQUIRED)

# 安装资源目录到share路径
install(
  DIRECTORY
    worlds
    models
    launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本（赋予可执行权限）
install(
  PROGRAMS
    scripts/spawn_pallets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
