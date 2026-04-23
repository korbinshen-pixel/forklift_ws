#!/usr/bin/env python3
"""
spawn_pallets.py
ROS2节点：在仓库货架区域随机生成3~6个托盘

功能：
  1. 在仓库地图的通道旁合理位置随机放置3~6个托盘
  2. 调用 /spawn_entity Gazebo服务生成托盘模型
  3. 随机朝向（0/90/180/270度）
  4. 记录生成的托盘列表（名称和位姿）
  5. 发布 /spawned_pallets topic（geometry_msgs/PoseArray）

使用方法：
  ros2 run forklift_gazebo spawn_pallets.py
  # 或
  python3 spawn_pallets.py
"""

import os
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Pose, PoseArray
from gazebo_msgs.srv import SpawnEntity

from ament_index_python.packages import get_package_share_directory


class PalletSpawner(Node):
    """托盘生成器节点"""

    def __init__(self):
        super().__init__('pallet_spawner')

        # ==================== 参数声明 ====================
        # 生成托盘数量范围
        self.declare_parameter('min_pallets', 3)
        self.declare_parameter('max_pallets', 6)
        # 随机种子（-1表示随机）
        self.declare_parameter('random_seed', -1)

        min_pallets = self.get_parameter('min_pallets').value
        max_pallets = self.get_parameter('max_pallets').value
        random_seed = self.get_parameter('random_seed').value

        # 设置随机种子
        if random_seed >= 0:
            random.seed(random_seed)
            self.get_logger().info(f'使用固定随机种子: {random_seed}')
        else:
            self.get_logger().info('使用随机种子')

        # 生成托盘数量
        self.num_pallets = random.randint(min_pallets, max_pallets)
        self.get_logger().info(f'计划生成 {self.num_pallets} 个托盘')

        # ==================== 托盘模型SDF路径 ====================
        # 从包路径获取模型文件
        pkg_dir = get_package_share_directory('forklift_gazebo')
        sdf_path = os.path.join(pkg_dir, 'models', 'pallet', 'model.sdf')

        if not os.path.exists(sdf_path):
            self.get_logger().error(f'托盘SDF文件不存在: {sdf_path}')
            raise FileNotFoundError(f'Pallet SDF not found: {sdf_path}')

        with open(sdf_path, 'r') as f:
            self.pallet_sdf = f.read()

        self.get_logger().info(f'已加载托盘模型: {sdf_path}')

        # ==================== 发布话题 ====================
        # 使用TRANSIENT_LOCAL确保后来的订阅者也能收到消息
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/spawned_pallets',
            qos
        )

        # ==================== Gazebo spawn服务客户端 ====================
        self.spawn_client = self.create_service_client_with_wait(
            SpawnEntity,
            '/spawn_entity'
        )

        # ==================== 定义可放置托盘的位置区域 ====================
        # 仓库布局：货架在X=±5.5和X=±7.0，Y从-7.5到7.5
        # 通道区域：X=-4到X=4（6m宽中央通道）
        # 货架前通道（托盘放置区）：货架前方约0.8m处
        #
        # 托盘合理放置位置（通道旁，货架前/侧边）：
        self.spawn_zones = [
            # 东侧货架区前通道（X=3.5~4.5，沿Y方向排列）
            {'x_range': (3.8, 4.8), 'y_range': (1.0, 8.5),  'desc': '东侧通道北区'},
            {'x_range': (3.8, 4.8), 'y_range': (-8.5, -1.0), 'desc': '东侧通道南区'},
            # 西侧货架区前通道（X=-3.5~-4.5）
            {'x_range': (-4.8, -3.8), 'y_range': (1.0, 8.5),  'desc': '西侧通道北区'},
            {'x_range': (-4.8, -3.8), 'y_range': (-8.5, -1.0), 'desc': '西侧通道南区'},
            # 中央主通道（X=-2~2，Y从1到8.5）
            {'x_range': (-2.0, 2.0), 'y_range': (1.5, 8.5),  'desc': '中央通道北区'},
            # 南侧入口区
            {'x_range': (-2.0, 2.0), 'y_range': (-8.5, -5.0), 'desc': '南侧入口区'},
        ]

        # 已生成托盘的位置记录（用于避免重叠）
        self.spawned_positions = []

        # 生成托盘
        self.spawned_pallets = []
        self.spawn_all_pallets()

    def create_service_client_with_wait(self, srv_type, srv_name, timeout_sec=30.0):
        """创建服务客户端并等待服务可用"""
        client = self.create_client(srv_type, srv_name)
        self.get_logger().info(f'等待服务 {srv_name} 就绪...')

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'服务 {srv_name} 在 {timeout_sec}s 内未就绪！')
            raise RuntimeError(f'Service {srv_name} not available')

        self.get_logger().info(f'服务 {srv_name} 已就绪')
        return client

    def generate_random_pose(self, attempt=0) -> Pose:
        """
        在可放置区域随机生成一个托盘位姿
        
        Returns:
            Pose: 生成的位姿（如果找不到合适位置返回None）
        """
        max_attempts = 50  # 最大尝试次数
        min_distance = 1.5  # 托盘之间的最小间距（m）

        for _ in range(max_attempts):
            # 随机选择一个放置区
            zone = random.choice(self.spawn_zones)

            # 在区域内随机选取X,Y坐标
            x = random.uniform(zone['x_range'][0], zone['x_range'][1])
            y = random.uniform(zone['y_range'][0], zone['y_range'][1])

            # 检查与已有托盘的距离
            too_close = False
            for pos in self.spawned_positions:
                dist = math.sqrt((x - pos[0])**2 + (y - pos[1])**2)
                if dist < min_distance:
                    too_close = True
                    break

            if not too_close:
                # 随机朝向：0/90/180/270度
                yaw_choices = [0.0, math.pi/2, math.pi, 3*math.pi/2]
                yaw = random.choice(yaw_choices)

                # 创建Pose
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.05  # 略高于地面，避免穿透

                # 四元数（绕Z轴旋转yaw角）
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = math.sin(yaw / 2.0)
                pose.orientation.w = math.cos(yaw / 2.0)

                # 记录位置
                self.spawned_positions.append((x, y))

                self.get_logger().info(
                    f'生成位置: ({x:.2f}, {y:.2f}, 0.05), '
                    f'朝向: {math.degrees(yaw):.0f}度, '
                    f'区域: {zone["desc"]}'
                )
                return pose

        # 找不到合适位置时使用中央通道
        self.get_logger().warning(f'无法找到合适位置，使用默认位置（尝试{attempt}）')
        pose = Pose()
        pose.position.x = float(attempt * 1.5 - 3.0)
        pose.position.y = 5.0
        pose.position.z = 0.05
        pose.orientation.w = 1.0
        return pose

    def spawn_pallet(self, pallet_name: str, pose: Pose) -> bool:
        """
        调用Gazebo /spawn_entity 服务生成一个托盘
        
        Args:
            pallet_name: 托盘实体名称
            pose: 生成位姿
            
        Returns:
            bool: 是否成功生成
        """
        request = SpawnEntity.Request()
        request.name = pallet_name
        request.xml = self.pallet_sdf
        request.robot_namespace = ''
        request.initial_pose = pose
        request.reference_frame = 'world'

        # 异步调用服务
        future = self.spawn_client.call_async(request)

        # 等待响应（最多10秒）
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f'成功生成托盘: {pallet_name}')
                return True
            else:
                self.get_logger().error(
                    f'生成托盘失败 {pallet_name}: {result.status_message}'
                )
                return False
        else:
            self.get_logger().error(f'生成托盘超时: {pallet_name}')
            return False

    def spawn_all_pallets(self):
        """生成所有托盘并发布位姿列表"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'

        self.get_logger().info(f'开始生成 {self.num_pallets} 个托盘...')

        for i in range(self.num_pallets):
            pallet_name = f'pallet_{i:03d}'
            pose = self.generate_random_pose(attempt=i)

            success = self.spawn_pallet(pallet_name, pose)

            if success:
                self.spawned_pallets.append({
                    'name': pallet_name,
                    'pose': pose
                })
                pose_array.poses.append(pose)

        # 发布生成的托盘位姿列表
        self.pose_array_pub.publish(pose_array)

        # 打印汇总报告
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'托盘生成完成！成功: {len(self.spawned_pallets)}/{self.num_pallets}')
        self.get_logger().info('托盘列表：')
        for pallet in self.spawned_pallets:
            p = pallet['pose'].position
            self.get_logger().info(
                f'  - {pallet["name"]}: '
                f'位置=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})'
            )
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'已发布托盘位姿到话题: /spawned_pallets '
            f'(geometry_msgs/PoseArray)'
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = PalletSpawner()
        # 短暂spin确保消息发布出去
        rclpy.spin_once(node, timeout_sec=2.0)
        node.get_logger().info('托盘生成节点完成，退出。')
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
