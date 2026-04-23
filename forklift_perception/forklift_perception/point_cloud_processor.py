#!/usr/bin/env python3
# =============================================================================
# point_cloud_processor.py
# 点云处理节点
#
# 功能：
#   - 订阅原始点云 /camera/depth/points
#   - 体素下采样（降低点云密度）
#   - 直通滤波（去除超出范围的点）
#   - RANSAC平面检测（分离地面）
#   - 发布处理后的点云 /processed_cloud
#   - 发布地面平面参数 /ground_plane
#
# 依赖：numpy（必须），open3d（可选，用于体素下采样和RANSAC）
# =============================================================================

import math
import struct
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ROS2 消息类型
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension

# 尝试导入 open3d（高性能点云处理库）
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False

# 尝试导入 sensor_msgs_py（更高效的PointCloud2解析）
try:
    from sensor_msgs_py import point_cloud2 as pc2_utils
    PC2_UTILS_AVAILABLE = True
except ImportError:
    PC2_UTILS_AVAILABLE = False


# =============================================================================
# PointCloud2 辅助函数（不依赖 sensor_msgs_py）
# =============================================================================
def decode_pointcloud2(cloud_msg: PointCloud2) -> np.ndarray:
    """
    将 PointCloud2 消息解析为 numpy array (N, 3)，仅提取 XYZ。
    支持 float32 和 float64 类型的 XYZ 字段。

    Returns:
        points: np.ndarray, shape (N, 3), dtype float32
                nan 表示无效点
    """
    # 解析字段偏移
    field_map = {}
    for field in cloud_msg.fields:
        field_map[field.name] = field

    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        raise ValueError('PointCloud2 消息缺少 x/y/z 字段')

    x_off = field_map['x'].offset
    y_off = field_map['y'].offset
    z_off = field_map['z'].offset

    # 判断数据类型（PointField.FLOAT32 = 7）
    dtype_char = 'f' if field_map['x'].datatype == PointField.FLOAT32 else 'd'
    dtype_size = 4   if dtype_char == 'f' else 8

    n_points = cloud_msg.width * cloud_msg.height
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    step = cloud_msg.point_step
    raw  = bytes(cloud_msg.data)

    # 向量化解析（numpy fromstring）
    try:
        # 将原始字节转为uint8，然后按 point_step 切割
        raw_arr = np.frombuffer(raw, dtype=np.uint8).reshape(n_points, step)
        xs = np.frombuffer(raw_arr[:, x_off:x_off+dtype_size].tobytes(), dtype=np.float32)
        ys = np.frombuffer(raw_arr[:, y_off:y_off+dtype_size].tobytes(), dtype=np.float32)
        zs = np.frombuffer(raw_arr[:, z_off:z_off+dtype_size].tobytes(), dtype=np.float32)
        points = np.column_stack([xs, ys, zs])
    except Exception:
        # 降级：逐点解析
        fmt = dtype_char
        points = np.zeros((n_points, 3), dtype=np.float32)
        for i in range(n_points):
            base = i * step
            try:
                points[i, 0] = struct.unpack_from(fmt, raw, base + x_off)[0]
                points[i, 1] = struct.unpack_from(fmt, raw, base + y_off)[0]
                points[i, 2] = struct.unpack_from(fmt, raw, base + z_off)[0]
            except struct.error:
                points[i] = np.nan

    return points.astype(np.float32)


def encode_pointcloud2(
    points: np.ndarray,
    frame_id: str,
    stamp
) -> PointCloud2:
    """
    将 numpy array (N, 3) 编码为 PointCloud2 消息。

    Args:
        points: (N, 3) float32 数组
        frame_id: 坐标系名称
        stamp: ROS时间戳

    Returns:
        PointCloud2 消息
    """
    cloud_msg = PointCloud2()
    cloud_msg.header.frame_id = frame_id
    cloud_msg.header.stamp    = stamp

    cloud_msg.height = 1
    cloud_msg.width  = len(points)
    cloud_msg.is_dense = False

    # 定义字段
    cloud_msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    cloud_msg.point_step = 12  # 3 * 4 bytes
    cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width
    cloud_msg.is_bigendian = False

    # 编码数据
    cloud_msg.data = points.astype(np.float32).tobytes()
    return cloud_msg


# =============================================================================
# PointCloudProcessorNode
# =============================================================================
class PointCloudProcessorNode(Node):
    """
    点云处理节点。
    对深度相机输出的原始点云进行下采样、滤波、地面分割处理。
    """

    def __init__(self):
        super().__init__('point_cloud_processor')

        # ---- 参数声明 ----
        # 体素下采样
        self.declare_parameter('voxel_size',          0.05)   # 体素边长(m)
        self.declare_parameter('use_voxel_downsample', True)  # 是否启用体素下采样

        # 直通滤波
        self.declare_parameter('passthrough_x_min',  -5.0)   # X轴最小值(m)
        self.declare_parameter('passthrough_x_max',   5.0)   # X轴最大值(m)
        self.declare_parameter('passthrough_y_min',  -3.0)   # Y轴最小值(m)（相机朝前Y为下）
        self.declare_parameter('passthrough_y_max',   3.0)   # Y轴最大值(m)
        self.declare_parameter('passthrough_z_min',   0.1)   # Z轴最小值(m)（近端截断）
        self.declare_parameter('passthrough_z_max',   8.0)   # Z轴最大值(m)（远端截断）

        # 地面平面检测（RANSAC）
        self.declare_parameter('ground_removal_enabled', True)    # 是否去除地面
        self.declare_parameter('ransac_distance_thresh', 0.05)    # RANSAC内点距离阈值(m)
        self.declare_parameter('ransac_num_iterations',  100)     # RANSAC迭代次数
        self.declare_parameter('ground_height_threshold', 0.1)    # 地面高度容差(m)

        # 其他
        self.declare_parameter('process_rate',        10.0)    # 处理频率上限(Hz)
        self.declare_parameter('input_topic',         '/camera/depth/points')
        self.declare_parameter('output_topic',        '/processed_cloud')
        self.declare_parameter('ground_plane_topic',  '/ground_plane')

        # 读取参数
        self.voxel_size         = self.get_parameter('voxel_size').value
        self.use_voxel          = self.get_parameter('use_voxel_downsample').value
        self.x_min = self.get_parameter('passthrough_x_min').value
        self.x_max = self.get_parameter('passthrough_x_max').value
        self.y_min = self.get_parameter('passthrough_y_min').value
        self.y_max = self.get_parameter('passthrough_y_max').value
        self.z_min = self.get_parameter('passthrough_z_min').value
        self.z_max = self.get_parameter('passthrough_z_max').value
        self.ground_removal     = self.get_parameter('ground_removal_enabled').value
        self.ransac_dist        = self.get_parameter('ransac_distance_thresh').value
        self.ransac_iters       = self.get_parameter('ransac_num_iterations').value
        self.ground_thresh      = self.get_parameter('ground_height_threshold').value
        self.process_rate       = self.get_parameter('process_rate').value
        input_topic             = self.get_parameter('input_topic').value
        output_topic            = self.get_parameter('output_topic').value
        ground_topic            = self.get_parameter('ground_plane_topic').value

        # 内部状态
        self._latest_cloud: Optional[PointCloud2] = None  # 最新点云
        self._last_process_time = 0.0                      # 上次处理时间戳
        self._process_interval  = 1.0 / self.process_rate  # 处理间隔(s)
        self._process_count = 0                            # 处理计数

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ---- 订阅：原始点云 ----
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self._cloud_callback,
            qos_profile=sensor_qos
        )

        # ---- 发布：处理后点云 ----
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        # ---- 发布：地面平面参数 ----
        # 格式: Float32MultiArray [a, b, c, d] -> 平面方程 ax+by+cz+d=0
        self._ground_pub = self.create_publisher(
            Float32MultiArray,
            ground_topic,
            10
        )

        # ---- 定时器：处理主循环 ----
        self._process_timer = self.create_timer(
            self._process_interval,
            self._process_loop
        )

        self.get_logger().info('point_cloud_processor 节点已启动')
        self.get_logger().info(f'  输入:  {input_topic}')
        self.get_logger().info(f'  输出:  {output_topic}')
        self.get_logger().info(f'  地面:  {ground_topic}')
        self.get_logger().info(f'  体素大小: {self.voxel_size}m')
        self.get_logger().info(
            f'  直通滤波: '
            f'X[{self.x_min},{self.x_max}] '
            f'Y[{self.y_min},{self.y_max}] '
            f'Z[{self.z_min},{self.z_max}]'
        )
        self.get_logger().info(f'  地面去除: {"开启" if self.ground_removal else "关闭"}')
        if OPEN3D_AVAILABLE:
            self.get_logger().info('  Open3D 可用，使用高性能点云处理')
        else:
            self.get_logger().info('  Open3D 不可用，使用 NumPy 实现')

    # =========================================================================
    # 订阅回调
    # =========================================================================
    def _cloud_callback(self, msg: PointCloud2):
        """接收原始点云"""
        self._latest_cloud = msg

    # =========================================================================
    # 处理主循环
    # =========================================================================
    def _process_loop(self):
        """
        定时器触发的处理循环：
        1. 解码原始点云
        2. 直通滤波
        3. 体素下采样
        4. 地面分割
        5. 发布结果
        """
        if self._latest_cloud is None:
            return

        cloud_msg = self._latest_cloud
        self._latest_cloud = None  # 清除，避免重复处理

        t_start = time.time()

        try:
            # ---- 步骤1：解码点云 ----
            if OPEN3D_AVAILABLE:
                points = self._decode_with_numpy(cloud_msg)
            else:
                points = decode_pointcloud2(cloud_msg)

            if points is None or len(points) == 0:
                return

            n_raw = len(points)

            # 移除 NaN 和 Inf
            valid_mask = np.isfinite(points).all(axis=1)
            points = points[valid_mask]
            if len(points) == 0:
                return

            # ---- 步骤2：直通滤波（去除范围外的点） ----
            points = self._passthrough_filter(points)
            n_after_pass = len(points)
            if len(points) == 0:
                return

            # ---- 步骤3：体素下采样 ----
            if self.use_voxel and len(points) > 1000:
                points = self._voxel_downsample(points)
            n_after_voxel = len(points)

            # ---- 步骤4：地面分割 ----
            ground_plane = None
            if self.ground_removal and len(points) > 50:
                points, ground_plane = self._remove_ground(points)

            # ---- 步骤5：发布处理后点云 ----
            now = self.get_clock().now().to_msg()
            processed_msg = encode_pointcloud2(
                points,
                cloud_msg.header.frame_id,
                now
            )
            self._cloud_pub.publish(processed_msg)

            # ---- 步骤6：发布地面平面参数 ----
            if ground_plane is not None:
                plane_msg = Float32MultiArray()
                plane_msg.layout.dim.append(
                    MultiArrayDimension(label='plane', size=4, stride=4)
                )
                plane_msg.data = [float(v) for v in ground_plane]
                self._ground_pub.publish(plane_msg)

            # ---- 日志（每50次打印一次统计）----
            self._process_count += 1
            if self._process_count % 50 == 0:
                t_elapsed = (time.time() - t_start) * 1000
                self.get_logger().info(
                    f'点云处理: '
                    f'原始={n_raw} -> '
                    f'直通后={n_after_pass} -> '
                    f'降采样后={n_after_voxel} -> '
                    f'去地面后={len(points)} | '
                    f'耗时={t_elapsed:.1f}ms'
                )

        except Exception as e:
            self.get_logger().error(f'点云处理出错: {e}')

    # =========================================================================
    # 直通滤波
    # =========================================================================
    def _passthrough_filter(self, points: np.ndarray) -> np.ndarray:
        """
        直通滤波：仅保留在指定范围内的点。
        适用于仓库环境：去除过近/过远、过高/过低的点。

        Args:
            points: (N, 3) float32，相机坐标系（x右, y下, z前）

        Returns:
            过滤后的点云 (M, 3)
        """
        mask = (
            (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max) &
            (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max) &
            (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        )
        return points[mask]

    # =========================================================================
    # 体素下采样
    # =========================================================================
    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        体素网格下采样：将点云划分为体素，每个体素取重心。
        降低点云密度，提高后续算法速度。

        Args:
            points: (N, 3) float32
            voxel_size: 体素边长(m)

        Returns:
            下采样后的点云 (M, 3)，M << N
        """
        if OPEN3D_AVAILABLE:
            # 使用 Open3D 高效实现
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            return np.asarray(pcd_down.points, dtype=np.float32)

        # NumPy 实现：将空间离散化为体素格，取每格中点的均值
        # 计算体素索引
        vmin = points.min(axis=0)
        indices = ((points - vmin) / self.voxel_size).astype(np.int32)

        # 使用字典聚合（仅适合中等规模点云）
        voxel_dict = {}
        for i, idx in enumerate(map(tuple, indices)):
            if idx not in voxel_dict:
                voxel_dict[idx] = []
            voxel_dict[idx].append(points[i])

        # 计算每个体素的重心
        downsampled = np.array([
            np.mean(pts, axis=0) for pts in voxel_dict.values()
        ], dtype=np.float32)

        return downsampled

    # =========================================================================
    # 地面分割（RANSAC平面检测）
    # =========================================================================
    def _remove_ground(
        self,
        points: np.ndarray
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        使用 RANSAC 检测并去除地面平面。
        在相机坐标系中，地面通常是 Y 方向最大的平面。

        Args:
            points: (N, 3) float32

        Returns:
            (non_ground_points, plane_coeffs)
            plane_coeffs: [a, b, c, d] 满足 ax+by+cz+d=0，None表示未检测到
        """
        if OPEN3D_AVAILABLE:
            return self._remove_ground_open3d(points)
        else:
            return self._remove_ground_ransac_numpy(points)

    def _remove_ground_open3d(
        self,
        points: np.ndarray
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """使用 Open3D RANSAC 检测地面"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

        # RANSAC 平面拟合
        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=self.ransac_dist,
            ransac_n=3,
            num_iterations=self.ransac_iters
        )

        [a, b, c, d] = plane_model

        # 验证是否为地面（法向量应接近 Y 轴，即 (0, 1, 0) 在相机坐标系）
        normal = np.array([a, b, c])
        normal /= np.linalg.norm(normal)
        y_axis = np.array([0.0, 1.0, 0.0])
        dot = abs(np.dot(normal, y_axis))

        if dot < 0.5:
            # 法向量与Y轴夹角超过60°，不是地面
            return points, None

        # 提取非地面点
        outlier_cloud = pcd.select_by_index(inlier_indices, invert=True)
        non_ground = np.asarray(outlier_cloud.points, dtype=np.float32)

        return non_ground, np.array([a, b, c, d], dtype=np.float32)

    def _remove_ground_ransac_numpy(
        self,
        points: np.ndarray
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        NumPy 实现的 RANSAC 地面平面检测。
        适合没有 Open3D 的环境，性能略低。
        """
        n = len(points)
        if n < 3:
            return points, None

        best_inliers = None
        best_plane   = None
        best_count   = 0

        rng = np.random.default_rng(seed=42)

        for _ in range(self.ransac_iters):
            # 随机选3个点
            idx = rng.choice(n, 3, replace=False)
            p1, p2, p3 = points[idx[0]], points[idx[1]], points[idx[2]]

            # 计算平面法向量
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue  # 三点共线，重新采样
            normal = normal / norm

            # 平面方程：ax+by+cz+d=0，d = -normal·p1
            a, b, c = normal
            d = -np.dot(normal, p1)

            # 计算所有点到平面的距离
            distances = np.abs(points @ normal + d)

            # 内点判断
            inliers = distances < self.ransac_dist
            count   = inliers.sum()

            if count > best_count:
                best_count   = count
                best_inliers = inliers
                best_plane   = np.array([a, b, c, d], dtype=np.float32)

        if best_plane is None or best_count < 50:
            return points, None

        # 验证是否为地面（法向量应接近 Y 轴）
        a, b, c, _ = best_plane
        normal = np.array([a, b, c])
        dot = abs(np.dot(normal, np.array([0.0, 1.0, 0.0])))
        if dot < 0.5:
            return points, None

        # 返回非地面点
        non_ground = points[~best_inliers]
        return non_ground, best_plane

    # =========================================================================
    # 辅助方法
    # =========================================================================
    def _decode_with_numpy(self, cloud_msg: PointCloud2) -> np.ndarray:
        """使用 NumPy 解码点云（备用方法）"""
        return decode_pointcloud2(cloud_msg)


# =============================================================================
# Main
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('point_cloud_processor 节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
