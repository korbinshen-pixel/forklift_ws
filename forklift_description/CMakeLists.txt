cmake_minimum_required(VERSION 3.8)
project(forklift_description)

# 查找ament_cmake依赖
find_package(ament_cmake REQUIRED)

# 安装资源目录到share路径
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# 安装package.xml
ament_package()
