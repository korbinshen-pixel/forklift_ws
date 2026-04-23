"""
forklift_world.launch.py
启动叉车仓库仿真环境

启动内容：
  1. Gazebo 仿真器（加载 warehouse.world）
  2. 生成叉车机器人到 Gazebo
  3. 启动 robot_state_publisher
  4. （可选）运行 spawn_pallets 脚本

使用方法：
  ros2 launch forklift_gazebo forklift_world.launch.py
  ros2 launch forklift_gazebo forklift_world.launch.py spawn_pallets:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== 包路径 ====================
    pkg_gazebo = get_package_share_directory('forklift_gazebo')
    pkg_description = get_package_share_directory('forklift_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # world文件路径
    world_file = os.path.join(pkg_gazebo, 'worlds', 'warehouse.world')

    # URDF xacro文件路径
    xacro_file = os.path.join(pkg_description, 'urdf', 'forklift.urdf.xacro')

    # ==================== 声明Launch参数 ====================
    # 是否使用仿真时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用Gazebo仿真时间'
    )

    # 是否在仿真启动后生成托盘
    spawn_pallets_arg = DeclareLaunchArgument(
        'spawn_pallets',
        default_value='false',
        description='是否自动生成托盘（需要仓库环境已加载）'
    )

    # 叉车初始X位置
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='叉车初始X坐标（m）'
    )

    # 叉车初始Y位置
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='-8.0',
        description='叉车初始Y坐标（m），默认在仓库南侧入口'
    )

    # 叉车初始Z位置
    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='0.3',
        description='叉车初始Z坐标（m），略高于地面防止穿透'
    )

    # Gazebo是否以headless模式运行（无GUI）
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否以无GUI模式运行Gazebo（服务器模式）'
    )

    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_pallets = LaunchConfiguration('spawn_pallets')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    headless = LaunchConfiguration('headless')

    # ==================== 解析URDF ====================
    # 使用xacro处理URDF文件
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ==================== 启动Gazebo ====================
    # 包含gazebo_ros的launch文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            # headless模式下只启动gzserver
            'server_required': 'true',
            'gui': 'true',
            'force_system': 'false',
        }.items()
    )

    # Headless模式：只启动gzserver（无GUI）
    gzserver_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
        condition=IfCondition(headless)
    )

    # ==================== robot_state_publisher ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }]
    )

    # ==================== 在Gazebo中生成叉车 ====================
    # 使用gazebo_ros的spawn_entity.py脚本
    # 延迟5秒等待Gazebo完全启动
    spawn_forklift = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_forklift',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',   # 从话题读取URDF
                    '-entity', 'forklift',             # 实体名称
                    '-x', robot_x,                    # 初始X坐标
                    '-y', robot_y,                    # 初始Y坐标
                    '-z', robot_z,                    # 初始Z坐标
                    '-Y', '1.5708',                   # 初始偏航角（朝向仓库内部，90度）
                    '-unpause',                       # 生成后恢复仿真
                ]
            )
        ]
    )

    # ==================== 生成托盘（可选）====================
    # 延迟10秒，等待叉车和仓库完全加载
    spawn_pallets_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='forklift_gazebo',
                executable='spawn_pallets.py',
                name='pallet_spawner',
                output='screen',
                parameters=[{
                    'min_pallets': 3,
                    'max_pallets': 6,
                    'random_seed': -1,  # -1表示随机种子
                    'use_sim_time': use_sim_time,
                }],
                condition=IfCondition(spawn_pallets)
            )
        ]
    )

    # ==================== 关节状态发布器（可选）====================
    # 在仿真中由差速驱动插件和Gazebo提供关节状态
    # 此处作为备用，可被注释掉
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        # Launch参数声明
        use_sim_time_arg,
        spawn_pallets_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        headless_arg,

        # 启动Gazebo仿真器（正常模式）
        gazebo_launch,

        # 启动robot_state_publisher
        robot_state_publisher,

        # 关节状态发布（Gazebo插件会覆盖）
        joint_state_publisher,

        # 延迟生成叉车
        spawn_forklift,

        # 延迟生成托盘（如果启用）
        spawn_pallets_node,
    ])
