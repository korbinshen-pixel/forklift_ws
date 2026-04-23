#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
叉车仿真系统一键启动文件
========================
启动顺序：
  1. Gazebo + 仓库 world (forklift_gazebo)
  2. robot_state_publisher（叉车 URDF 状态发布）
  3. Nav2 导航栈（forklift_navigation）
  4. 托盘检测节点（forklift_perception）
  5. 点云处理节点
  6. 任务管理节点（forklift_task_manager）
  7. 货叉控制节点（fork_controller）
  8. 托盘生成脚本（延迟 5s 后执行）
  9. RViz2（带自定义配置）

使用方法：
  ros2 launch forklift_bringup forklift_full.launch.py
  ros2 launch forklift_bringup forklift_full.launch.py use_rviz:=false
  ros2 launch forklift_bringup forklift_full.launch.py world_name:=warehouse_large.world

ROS2 Humble + Gazebo 11.10.2
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成 LaunchDescription"""

    # ============================================================
    # 包路径获取
    # ============================================================
    pkg_forklift_bringup    = get_package_share_directory('forklift_bringup')
    pkg_forklift_gazebo     = get_package_share_directory('forklift_gazebo')
    pkg_forklift_navigation = get_package_share_directory('forklift_navigation')
    pkg_forklift_perception = get_package_share_directory('forklift_perception')
    pkg_forklift_control    = get_package_share_directory('forklift_control')
    pkg_forklift_desc       = get_package_share_directory('forklift_description')

    # RViz 配置文件路径
    rviz_config_file = os.path.join(
        pkg_forklift_bringup, 'config', 'rviz_config.rviz'
    )

    # ============================================================
    # 启动参数声明
    # ============================================================
    # 是否启动 RViz2
    arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 RViz2 可视化界面'
    )
    # 是否启动仿真（false 则只启动控制节点，用于真机）
    arg_use_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟（Gazebo）'
    )
    # 世界文件名
    arg_world = DeclareLaunchArgument(
        'world_name',
        default_value='warehouse.world',
        description='Gazebo 世界文件名（位于 forklift_gazebo/worlds/ 目录）'
    )
    # 叉车初始位置
    arg_init_x = DeclareLaunchArgument(
        'init_x', default_value='-3.0',
        description='叉车初始 X 坐标（米）'
    )
    arg_init_y = DeclareLaunchArgument(
        'init_y', default_value='0.0',
        description='叉车初始 Y 坐标（米）'
    )
    arg_init_yaw = DeclareLaunchArgument(
        'init_yaw', default_value='0.0',
        description='叉车初始偏航角（弧度）'
    )
    # 是否启动自动任务（启动后自动触发取货）
    arg_auto_task = DeclareLaunchArgument(
        'auto_task',
        default_value='false',
        description='是否自动触发搬运任务'
    )
    # 调试模式（输出更多日志）
    arg_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='调试模式'
    )

    # 读取启动配置
    use_rviz    = LaunchConfiguration('use_rviz')
    use_sim     = LaunchConfiguration('use_sim_time')
    world_name  = LaunchConfiguration('world_name')
    init_x      = LaunchConfiguration('init_x')
    init_y      = LaunchConfiguration('init_y')
    init_yaw    = LaunchConfiguration('init_yaw')

    # ============================================================
    # 1. Gazebo 仿真环境
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_gazebo, 'launch', 'forklift_world.launch.py')
        ),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim,
            'init_x': init_x,
            'init_y': init_y,
            'init_yaw': init_yaw,
            'verbose': 'false',
        }.items()
    )

    # ============================================================
    # 2. robot_state_publisher（叉车 URDF + TF 树）
    # ============================================================
    # 读取 URDF 文件（通过 xacro 处理）
    urdf_file = os.path.join(pkg_forklift_desc, 'urdf', 'forklift.urdf.xacro')
    robot_description_cmd = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': use_sim,
        }]
    )

    # ============================================================
    # 3. Nav2 导航栈
    # ============================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim,
        }.items()
    )

    # ============================================================
    # 4. 托盘检测节点（forklift_perception）
    # ============================================================
    pallet_detector_node = Node(
        package='forklift_perception',
        executable='pallet_detector_node',
        name='pallet_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            # 检测置信度阈值
            'confidence_threshold': 0.6,
            # 输入点云话题
            'input_cloud_topic': '/camera/depth/points',
            # 输出托盘位姿话题
            'output_pallet_topic': '/pallet_pose',
        }]
    )

    # ============================================================
    # 5. 点云处理节点（深度图 → 点云转换/滤波）
    # ============================================================
    pointcloud_processor_node = Node(
        package='forklift_perception',
        executable='point_cloud_processor',
        name='pointcloud_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            # 点云下采样体素大小（米）
            'voxel_size': 0.02,
            # 直通滤波范围（米）
            'passthrough_min': 0.1,
            'passthrough_max': 5.0,
        }]
    )

    # ============================================================
    # 6. 任务管理节点（叉车状态机）
    # ============================================================
    task_manager_node = Node(
        package='forklift_control',
        executable='forklift_task_manager',
        name='forklift_task_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
        }],
        remappings=[
            # 如果话题名有前缀差异，在此配置重映射
            # ('/cmd_vel', '/forklift/cmd_vel'),
        ]
    )

    # ============================================================
    # 7. 货叉控制节点
    # ============================================================
    fork_controller_node = Node(
        package='forklift_control',
        executable='fork_controller',
        name='fork_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            # 货叉关节名（与 URDF 一致）
            'fork_joint_name': 'fork_lift_joint',
            # 最低高度（米）
            'fork_min_height': 0.0,
            # 最高高度（米）
            'fork_max_height': 1.5,
            # 最大移动速度（m/s）
            'max_velocity': 0.05,
            # 加速度（m/s²）
            'acceleration': 0.02,
        }]
    )

    # ============================================================
    # 8. 托盘生成脚本（延迟 5s 后执行，等待 Gazebo 完全启动）
    # ============================================================
    spawn_pallets_action = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[延迟5s后] 开始生成仓库托盘...'),
            Node(
                package='forklift_gazebo',
                executable='spawn_pallets',
                name='spawn_pallets',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim,
                    # 生成托盘数量
                    'num_pallets': 3,
                    # 托盘生成区域（随机分布）
                    'spawn_area_x_min': 1.0,
                    'spawn_area_x_max': 8.0,
                    'spawn_area_y_min': -3.0,
                    'spawn_area_y_max': 3.0,
                }]
            ),
        ]
    )

    # ============================================================
    # 9. RViz2（带自定义配置文件）
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(use_rviz)
    )

    # ============================================================
    # 自动任务触发（可选，延迟 15s 后调用 /start_pickup_task）
    # ============================================================
    auto_task_trigger = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[自动任务] 触发叉车搬运任务...'),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/start_pickup_task', 'std_srvs/srv/Trigger', '{}'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('auto_task'))
            ),
        ]
    )

    # ============================================================
    # 启动信息
    # ============================================================
    startup_log = LogInfo(
        msg='======== 叉车仿真系统启动中 ========'
    )
    ready_log = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='系统就绪！使用 "ros2 service call /start_pickup_task std_srvs/srv/Trigger {}" 触发任务')
        ]
    )

    # ============================================================
    # 组装 LaunchDescription
    # ============================================================
    return LaunchDescription([
        # --- 启动参数 ---
        arg_use_rviz,
        arg_use_sim,
        arg_world,
        arg_init_x,
        arg_init_y,
        arg_init_yaw,
        arg_auto_task,
        arg_debug,

        # --- 启动信息 ---
        startup_log,

        # --- 仿真基础设施 ---
        gazebo_launch,
        robot_state_publisher_node,

        # --- 导航 ---
        nav2_launch,

        # --- 感知 ---
        pallet_detector_node,
        pointcloud_processor_node,

        # --- 控制 ---
        task_manager_node,
        fork_controller_node,

        # --- 延迟任务 ---
        spawn_pallets_action,
        auto_task_trigger,

        # --- 可视化 ---
        rviz_node,

        # --- 就绪提示 ---
        ready_log,
    ])
