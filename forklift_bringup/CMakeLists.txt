cmake_minimum_required(VERSION 3.8)
project(forklift_bringup)

# 查找必要依赖
find_package(ament_cmake REQUIRED)

# 安装 launch 文件目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件目录（RViz 配置等）
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 resource 标记文件（ament 资源索引）
install(FILES
  resource/forklift_bringup
  DESTINATION share/ament_index/resource_index/packages
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
