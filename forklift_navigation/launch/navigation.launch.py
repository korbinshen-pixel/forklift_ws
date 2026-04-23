#!/usr/bin/env python3
# =============================================================================
# navigation.launch.py
# 叉车导航完整启动文件
# 功能：启动 Nav2 完整导航栈（Map Server + AMCL + 所有Nav2节点）
# 用法：ros2 launch forklift_navigation navigation.launch.py
#       ros2 launch forklift_navigation navigation.launch.py use_sim_time:=true
#       ros2 launch forklift_navigation navigation.launch.py slam:=true  # 建图模式
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    EnvironmentVariable,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 包路径
    # -------------------------------------------------------------------------
    pkg_nav  = get_package_share_directory('forklift_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # -------------------------------------------------------------------------
    # Launch 参数声明
    # -------------------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间（Gazebo仿真必须设为true）'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='是否启用SLAM建图模式（true=建图, false=定位导航）'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'warehouse_map.yaml'),
        description='地图YAML文件路径（仅在 slam:=false 时使用）'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Nav2参数文件路径'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
        description='SLAM Toolbox参数文件路径'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='自动启动Nav2生命周期节点'
    )

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='是否使用组合节点容器（提高性能）'
    )

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='节点崩溃后是否自动重启'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别: debug, info, warn, error'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='节点命名空间（多机器人场景使用）'
    )

    # -------------------------------------------------------------------------
    # LaunchConfiguration 变量
    # -------------------------------------------------------------------------
    use_sim_time      = LaunchConfiguration('use_sim_time')
    slam              = LaunchConfiguration('slam')
    map_yaml_file     = LaunchConfiguration('map')
    params_file       = LaunchConfiguration('params_file')
    slam_params_file  = LaunchConfiguration('slam_params_file')
    autostart         = LaunchConfiguration('autostart')
    use_composition   = LaunchConfiguration('use_composition')
    use_respawn       = LaunchConfiguration('use_respawn')
    log_level         = LaunchConfiguration('log_level')
    namespace         = LaunchConfiguration('namespace')

    # -------------------------------------------------------------------------
    # 环境变量：设置日志级别
    # -------------------------------------------------------------------------
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # -------------------------------------------------------------------------
    # SLAM Toolbox 节点（建图模式）
    # 条件：slam:=true 时启动
    # -------------------------------------------------------------------------
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # -------------------------------------------------------------------------
    # Map Server（地图服务器）
    # 条件：slam:=false 时启动（使用预建地图导航）
    # -------------------------------------------------------------------------
    map_server_node = Node(
        condition=UnlessCondition(slam),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file,
             'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # AMCL（自适应蒙特卡洛定位）
    # 条件：slam:=false 时启动（SLAM自带定位，不需要AMCL）
    # -------------------------------------------------------------------------
    amcl_node = Node(
        condition=UnlessCondition(slam),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Nav2 BT Navigator（行为树导航器）
    # -------------------------------------------------------------------------
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Planner Server（全局路径规划）
    # -------------------------------------------------------------------------
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Controller Server（局部路径控制/DWB）
    # -------------------------------------------------------------------------
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[]
    )

    # -------------------------------------------------------------------------
    # Smoother Server（路径平滑）
    # -------------------------------------------------------------------------
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Behavior Server（故障恢复行为）
    # -------------------------------------------------------------------------
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Waypoint Follower（路点跟随）
    # -------------------------------------------------------------------------
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # -------------------------------------------------------------------------
    # Velocity Smoother（速度平滑，防止叉车急停急起）
    # -------------------------------------------------------------------------
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # -------------------------------------------------------------------------
    # Lifecycle Manager（生命周期管理器）
    # 管理定位和地图相关节点
    # -------------------------------------------------------------------------
    lifecycle_manager_localization = Node(
        condition=UnlessCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            # 管理的节点列表（顺序启动）
            {'node_names': ['map_server', 'amcl']},
            {'bond_timeout': 4.0},
        ]
    )

    # SLAM模式的生命周期管理
    lifecycle_manager_slam = Node(
        condition=IfCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['slam_toolbox']},
            {'bond_timeout': 4.0},
        ]
    )

    # 管理导航相关节点
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ]
            },
            {'bond_timeout': 4.0},
        ]
    )

    # -------------------------------------------------------------------------
    # 叉车自定义导航节点（延迟启动，等待Nav2就绪）
    # -------------------------------------------------------------------------
    navigate_to_point_node = TimerAction(
        period=5.0,  # 延迟5秒，等待Nav2生命周期节点就绪
        actions=[
            Node(
                package='forklift_navigation',
                executable='navigate_to_point.py',
                name='navigate_to_point',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'goal_tolerance_xy': 0.25},
                    {'goal_tolerance_yaw': 0.25},
                ],
                arguments=['--ros-args', '--log-level', log_level],
            )
        ]
    )

    # -------------------------------------------------------------------------
    # 打印启动信息
    # -------------------------------------------------------------------------
    log_slam_mode = LogInfo(
        condition=IfCondition(slam),
        msg='[forklift_navigation] 启动模式: SLAM建图模式'
    )
    log_nav_mode = LogInfo(
        condition=UnlessCondition(slam),
        msg='[forklift_navigation] 启动模式: 定位导航模式（使用预建地图）'
    )

    # -------------------------------------------------------------------------
    # 返回 LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription([
        # 环境变量
        stdout_linebuf_envvar,

        # 参数声明
        declare_use_sim_time,
        declare_slam,
        declare_map_yaml,
        declare_params_file,
        declare_slam_params_file,
        declare_autostart,
        declare_use_composition,
        declare_use_respawn,
        declare_log_level,
        declare_namespace,

        # 日志
        log_slam_mode,
        log_nav_mode,

        # ---- 定位节点 ----
        # SLAM模式
        slam_toolbox_node,
        lifecycle_manager_slam,

        # 定位导航模式
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        # ---- Nav2核心节点 ----
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_navigation,

        # ---- 叉车自定义节点 ----
        navigate_to_point_node,
    ])
