"""
display.launch.py
启动叉车模型可视化：robot_state_publisher + joint_state_publisher_gui + rviz2

使用方法：
  ros2 launch forklift_description display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('forklift_description')

    # URDF xacro 文件路径
    xacro_file = os.path.join(pkg_dir, 'urdf', 'forklift.urdf.xacro')

    # RViz2 配置文件路径（如果存在）
    rviz_config_file = os.path.join(pkg_dir, 'config', 'display.rviz')

    # ========== 声明Launch参数 ==========
    # 是否使用仿真时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间（Gazebo中设置为true）'
    )

    # RViz2配置文件路径参数
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='RViz2配置文件路径'
    )

    # 获取Launch配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    # ========== 解析URDF ==========
    # 使用xacro命令处理xacro文件，生成URDF字符串
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ========== 节点定义 ==========

    # 1. robot_state_publisher: 将URDF发布到 /robot_description，并发布TF变换
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # 2. joint_state_publisher_gui: 提供GUI滑块控制关节状态（含货叉升降）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 3. rviz2: 可视化叉车模型
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ])
