#!/usr/bin/env python3
# =============================================================================
# pallet_detector_node.py
# 托盘检测节点 - 已训练模型推理接口
#
# 功能：
#   - 订阅 RGB图像、深度图像、点云
#   - 提供标准化模型推理接口（TODO区域供用户插入自己的模型）
#   - 发布 3D检测结果、最近托盘位姿
#   - 在TF树中广播托盘坐标系
#   - 仿真模式：随机生成假检测结果用于测试
#
# 话题：
#   订阅：
#     /camera/color/image_raw    (sensor_msgs/Image)
#     /camera/depth/image_raw    (sensor_msgs/Image)
#     /camera/depth/points       (sensor_msgs/PointCloud2)
#   发布：
#     /pallet_detections         (vision_msgs/Detection3DArray)
#     /pallet_pose               (geometry_msgs/PoseStamped)
#
# 参数：
#   use_sim_detection (bool, default: true)  - 使用仿真假检测
#   detection_confidence_threshold (float, default: 0.5) - 置信度阈值
# =============================================================================

import math
import random
import struct
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ROS2 消息类型
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
    BoundingBox3D,
)

# TF2
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

# cv_bridge：ROS Image <-> OpenCV
try:
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False


# =============================================================================
# 相机内参（默认值，应通过 /camera_info 话题获取或参数传入）
# =============================================================================
DEFAULT_FX = 525.0   # x方向焦距（像素）
DEFAULT_FY = 525.0   # y方向焦距（像素）
DEFAULT_CX = 319.5   # 主点 x（像素）
DEFAULT_CY = 239.5   # 主点 y（像素）


# =============================================================================
# PalletDetectorNode
# =============================================================================
class PalletDetectorNode(Node):
    """
    托盘检测节点。
    提供标准化的模型推理接口，支持仿真模式测试。
    """

    def __init__(self):
        super().__init__('pallet_detector')

        # ---- 参数声明 ----
        self.declare_parameter('use_sim_detection',              True)   # 仿真模式
        self.declare_parameter('detection_confidence_threshold', 0.5)    # 置信度阈值
        self.declare_parameter('camera_frame',                   'camera_color_optical_frame')  # 相机坐标系
        self.declare_parameter('base_frame',                     'base_link')   # 机器人基坐标系
        self.declare_parameter('max_detection_range',            5.0)    # 最大检测距离(m)
        self.declare_parameter('sim_detection_rate',             2.0)    # 仿真检测频率(Hz)
        self.declare_parameter('sim_num_pallets',                2)      # 仿真托盘数量
        # 相机内参（可覆盖）
        self.declare_parameter('camera_fx', DEFAULT_FX)
        self.declare_parameter('camera_fy', DEFAULT_FY)
        self.declare_parameter('camera_cx', DEFAULT_CX)
        self.declare_parameter('camera_cy', DEFAULT_CY)

        # 读取参数
        self.use_sim          = self.get_parameter('use_sim_detection').value
        self.conf_threshold   = self.get_parameter('detection_confidence_threshold').value
        self.camera_frame     = self.get_parameter('camera_frame').value
        self.base_frame       = self.get_parameter('base_frame').value
        self.max_range        = self.get_parameter('max_detection_range').value
        self.sim_rate         = self.get_parameter('sim_detection_rate').value
        self.sim_num_pallets  = self.get_parameter('sim_num_pallets').value
        self.fx               = self.get_parameter('camera_fx').value
        self.fy               = self.get_parameter('camera_fy').value
        self.cx               = self.get_parameter('camera_cx').value
        self.cy               = self.get_parameter('camera_cy').value

        # ---- cv_bridge ----
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn('cv_bridge 未安装，图像处理功能受限')

        # ---- TF2 广播器 ----
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- TF2 监听器（用于坐标变换） ----
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 内部状态 ----
        self._latest_color_image: Optional[Image] = None    # 最新彩色图像
        self._latest_depth_image: Optional[Image] = None    # 最新深度图像
        self._latest_pointcloud:  Optional[PointCloud2] = None  # 最新点云
        self._detection_count: int = 0                      # 检测计数（用于日志）

        # ---- 模型（用户插入区域） ----
        self._model = None          # 推理模型对象
        self._model_loaded = False  # 模型加载状态
        self._init_model()          # 初始化模型

        # ---- QoS 配置 ----
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ---- 订阅：彩色图像 ----
        self._color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._color_image_callback,
            qos_profile=sensor_qos
        )

        # ---- 订阅：深度图像 ----
        self._depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self._depth_image_callback,
            qos_profile=sensor_qos
        )

        # ---- 订阅：点云 ----
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self._pointcloud_callback,
            qos_profile=sensor_qos
        )

        # ---- 发布：3D检测结果数组 ----
        self._detection_pub = self.create_publisher(
            Detection3DArray,
            '/pallet_detections',
            10
        )

        # ---- 发布：最近托盘位姿 ----
        self._pallet_pose_pub = self.create_publisher(
            PoseStamped,
            '/pallet_pose',
            10
        )

        # ---- 定时器：检测主循环 ----
        if self.use_sim:
            # 仿真模式：按设定频率生成假检测
            self._detect_timer = self.create_timer(
                1.0 / self.sim_rate,
                self._sim_detection_loop
            )
            self.get_logger().info(
                f'[仿真模式] 启动，频率={self.sim_rate}Hz，'
                f'托盘数量={self.sim_num_pallets}，置信度阈值={self.conf_threshold}'
            )
        else:
            # 真实模式：每接收图像触发推理（通过回调驱动）
            self._detect_timer = self.create_timer(
                0.1,  # 10Hz 检查是否有新数据
                self._real_detection_loop
            )
            self.get_logger().info(
                f'[真实模式] 等待相机数据，置信度阈值={self.conf_threshold}'
            )

        self.get_logger().info('pallet_detector 节点已启动')
        self.get_logger().info(f'  彩色图像: /camera/color/image_raw')
        self.get_logger().info(f'  深度图像: /camera/depth/image_raw')
        self.get_logger().info(f'  点云:     /camera/depth/points')
        self.get_logger().info(f'  检测输出: /pallet_detections')
        self.get_logger().info(f'  托盘位姿: /pallet_pose')

    # =========================================================================
    # 模型初始化（用户插入区域）
    # =========================================================================
    def _init_model(self):
        """
        初始化推理模型。
        在此处加载你训练好的托盘检测模型。
        """
        if self.use_sim:
            # 仿真模式不需要真实模型
            self._model_loaded = True
            return

        # ======================================================================
        # TODO: 在此处插入模型初始化代码
        # ======================================================================
        # 示例1：加载 ONNX 模型（OpenCV DNN）
        # import cv2
        # model_path = '/path/to/pallet_detector.onnx'
        # self._model = cv2.dnn.readNetFromONNX(model_path)
        # self._model_loaded = True
        # self.get_logger().info(f'ONNX模型已加载: {model_path}')
        #
        # 示例2：加载 PyTorch 模型
        # import torch
        # from torchvision import transforms
        # model_path = '/path/to/pallet_detector.pt'
        # self._model = torch.load(model_path)
        # self._model.eval()
        # self._model_loaded = True
        #
        # 示例3：加载 TensorRT 引擎
        # import tensorrt as trt
        # engine_path = '/path/to/pallet_detector.engine'
        # ... (TensorRT 初始化代码)
        #
        # 示例4：使用 ultralytics YOLOv8
        # from ultralytics import YOLO
        # self._model = YOLO('/path/to/best.pt')
        # self._model_loaded = True
        # ======================================================================

        self.get_logger().warn(
            'TODO: 请在 _init_model() 中加载你的托盘检测模型！'
            '当前真实推理模式不可用。'
        )
        self._model_loaded = False

    # =========================================================================
    # 订阅回调：更新最新数据
    # =========================================================================
    def _color_image_callback(self, msg: Image):
        """接收彩色图像"""
        self._latest_color_image = msg

    def _depth_image_callback(self, msg: Image):
        """接收深度图像"""
        self._latest_depth_image = msg

    def _pointcloud_callback(self, msg: PointCloud2):
        """接收点云"""
        self._latest_pointcloud = msg

    # =========================================================================
    # 仿真检测主循环
    # =========================================================================
    def _sim_detection_loop(self):
        """
        仿真模式：在相机前方随机生成假托盘检测结果，用于测试下游逻辑。
        托盘布局：N个托盘，沿相机Z轴（前方）分布，左右交错。
        """
        now = self.get_clock().now().to_msg()
        detections_list = []

        for i in range(self.sim_num_pallets):
            # 在相机坐标系中生成随机位置
            # Z: 前方 1~5m（相机朝前为+Z）
            z = random.uniform(1.0, min(self.max_range, 5.0))
            # X: 左右 ±1.5m
            x = random.uniform(-1.5, 1.5)
            # Y: 略微偏下（托盘在地面）
            y = random.uniform(0.0, 0.5)

            # 随机置信度（0.5~1.0之间，模拟检测器输出）
            confidence = random.uniform(self.conf_threshold, 1.0)

            # 构造 Detection3D
            det = Detection3D()
            det.header.frame_id = self.camera_frame
            det.header.stamp    = now

            # 检测框（托盘标准尺寸：1.2m x 1.0m x 0.15m）
            det.bbox.center.position.x = x
            det.bbox.center.position.y = y
            det.bbox.center.position.z = z
            det.bbox.center.orientation.w = 1.0  # 无旋转
            det.bbox.size.x = 1.2    # 宽度(m)
            det.bbox.size.y = 0.15   # 高度(m)
            det.bbox.size.z = 1.0    # 深度(m)

            # 检测假设（类别 + 置信度）
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'pallet'
            hyp.hypothesis.score    = confidence
            hyp.pose.pose.position.x = x
            hyp.pose.pose.position.y = y
            hyp.pose.pose.position.z = z
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)

            detections_list.append(det)

        # 构造并发布 Detection3DArray
        self._publish_detections(detections_list, now, self.camera_frame)

    # =========================================================================
    # 真实检测主循环
    # =========================================================================
    def _real_detection_loop(self):
        """
        真实模式：等待相机数据，调用模型推理。
        需要模型已加载，且有最新彩色图像。
        """
        if not self._model_loaded:
            return  # 模型未加载，跳过

        if self._latest_color_image is None:
            return  # 没有图像数据，跳过

        # 获取最新图像
        color_img = self._latest_color_image
        depth_img = self._latest_depth_image
        cloud     = self._latest_pointcloud

        try:
            # 将 ROS Image 转换为 OpenCV 格式
            if self.bridge is not None:
                cv_color = self.bridge.imgmsg_to_cv2(color_img, 'bgr8')
                cv_depth = None
                if depth_img is not None:
                    cv_depth = self.bridge.imgmsg_to_cv2(
                        depth_img, desired_encoding='passthrough'
                    )
            else:
                self.get_logger().warn('cv_bridge 不可用，跳过推理', throttle_duration_sec=5.0)
                return

            # ==================================================================
            # TODO: 在此处插入模型推理代码
            # 输入：cv_color (numpy array, BGR, HxWx3)
            #       cv_depth (numpy array, float32, HxW, 单位：mm 或 m)
            # 输出：detections_2d - 二维检测框列表，格式如下：
            #   [
            #     {
            #       'bbox_2d': [x1, y1, x2, y2],  # 像素坐标
            #       'confidence': float,           # 置信度 0~1
            #       'class_id': str,               # 类别名称
            #     },
            #     ...
            #   ]
            # ==================================================================
            detections_2d = self._run_model_inference(cv_color)
            # ==================================================================

            # 过滤低置信度检测
            detections_2d = [
                d for d in detections_2d
                if d.get('confidence', 0.0) >= self.conf_threshold
            ]

            if not detections_2d:
                return

            now = self.get_clock().now().to_msg()
            detections_3d = []

            for det_2d in detections_2d:
                bbox   = det_2d['bbox_2d']          # [x1, y1, x2, y2]
                conf   = det_2d['confidence']
                cls_id = det_2d.get('class_id', 'pallet')

                # 计算边界框中心（像素坐标）
                cx_px = (bbox[0] + bbox[2]) / 2.0
                cy_px = (bbox[1] + bbox[3]) / 2.0

                # 融合深度信息，计算3D位置
                pos_3d = self._pixel_to_3d(cx_px, cy_px, cv_depth, cloud, color_img)
                if pos_3d is None:
                    continue  # 无法获取深度，跳过

                x3d, y3d, z3d = pos_3d

                # 过滤超出最大检测范围的目标
                if z3d > self.max_range or z3d <= 0:
                    continue

                # 构造 Detection3D
                det3d = Detection3D()
                det3d.header.frame_id = self.camera_frame
                det3d.header.stamp    = now
                det3d.bbox.center.position.x = x3d
                det3d.bbox.center.position.y = y3d
                det3d.bbox.center.position.z = z3d
                det3d.bbox.center.orientation.w = 1.0
                # 托盘标准尺寸（可从模型输出获取）
                det3d.bbox.size.x = 1.2
                det3d.bbox.size.y = 0.15
                det3d.bbox.size.z = 1.0

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_id
                hyp.hypothesis.score    = conf
                hyp.pose.pose.position.x = x3d
                hyp.pose.pose.position.y = y3d
                hyp.pose.pose.position.z = z3d
                hyp.pose.pose.orientation.w = 1.0
                det3d.results.append(hyp)
                detections_3d.append(det3d)

            self._publish_detections(detections_3d, now, self.camera_frame)

        except Exception as e:
            self.get_logger().error(f'检测推理出错: {e}')

    # =========================================================================
    # TODO: 模型推理接口（用户实现）
    # =========================================================================
    def _run_model_inference(self, cv_image: np.ndarray) -> List[dict]:
        """
        运行托盘检测模型推理。
        用户需要在此处实现具体的推理逻辑。

        Args:
            cv_image: OpenCV 彩色图像，BGR格式，numpy array (H, W, 3)

        Returns:
            List[dict]: 检测结果列表，每个元素包含：
                - 'bbox_2d': [x1, y1, x2, y2] 像素坐标
                - 'confidence': 置信度 (0.0 ~ 1.0)
                - 'class_id': 类别字符串（如 'pallet'）
        """
        # ======================================================================
        # TODO: 替换以下代码为真实模型推理
        # ======================================================================
        # 示例：使用 ONNX 模型（OpenCV DNN）
        # blob = cv2.dnn.blobFromImage(cv_image, 1/255.0, (640, 640), swapRB=True)
        # self._model.setInput(blob)
        # outputs = self._model.forward()
        # detections = self._parse_yolo_output(outputs, cv_image.shape)
        # return detections
        #
        # 示例：使用 ultralytics YOLO
        # results = self._model(cv_image)
        # detections = []
        # for r in results:
        #     for box in r.boxes:
        #         detections.append({
        #             'bbox_2d': box.xyxy[0].tolist(),
        #             'confidence': float(box.conf[0]),
        #             'class_id': r.names[int(box.cls[0])]
        #         })
        # return detections
        # ======================================================================

        # 占位返回（模型未实现时）
        return []

    # =========================================================================
    # 深度融合：像素坐标 -> 3D世界坐标
    # =========================================================================
    def _pixel_to_3d(
        self,
        u: float,
        v: float,
        depth_image: Optional[np.ndarray],
        cloud: Optional[PointCloud2],
        color_img: Image,
    ) -> Optional[Tuple[float, float, float]]:
        """
        将像素坐标 (u, v) 结合深度信息转换为相机坐标系下的3D坐标。
        优先使用深度图，其次使用点云。

        Args:
            u, v: 像素坐标
            depth_image: 深度图 (HxW, float32/uint16, 单位mm或m)
            cloud: 点云
            color_img: 原始彩色图像消息（获取尺寸）

        Returns:
            (x, y, z) 相机坐标系下的3D坐标(m)，失败返回None
        """
        # ---- 方法1：使用深度图（效率最高）----
        if depth_image is not None:
            try:
                h, w = depth_image.shape[:2]
                ui = int(np.clip(u, 0, w - 1))
                vi = int(np.clip(v, 0, h - 1))

                depth_val = float(depth_image[vi, ui])

                # 处理不同的深度单位（mm -> m 或直接 m）
                if depth_val > 100:
                    # 单位可能是毫米（RealSense默认）
                    depth_m = depth_val / 1000.0
                else:
                    depth_m = depth_val

                if depth_m <= 0 or not np.isfinite(depth_m):
                    return None

                # 使用相机内参反投影
                x = (u - self.cx) * depth_m / self.fx
                y = (v - self.cy) * depth_m / self.fy
                z = depth_m
                return (x, y, z)

            except Exception as e:
                self.get_logger().warn(f'深度图读取失败: {e}', throttle_duration_sec=2.0)

        # ---- 方法2：使用点云（精度更高，计算量大）----
        if cloud is not None:
            try:
                # 从点云中提取对应像素位置的3D点
                # PointCloud2 中点的索引 = row * width + col（有序点云）
                if cloud.is_dense:
                    return None  # 无序点云，无法直接索引

                col = int(np.clip(u, 0, cloud.width  - 1))
                row = int(np.clip(v, 0, cloud.height - 1))
                idx = row * cloud.width + col

                # 解析点云数据（XYZ格式，每点12字节）
                point_step = cloud.point_step
                offset = idx * point_step
                if offset + 12 > len(cloud.data):
                    return None

                x, y, z = struct.unpack_from('fff', bytes(cloud.data), offset)

                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    return None
                if z <= 0 or z > self.max_range:
                    return None

                return (float(x), float(y), float(z))

            except Exception as e:
                self.get_logger().warn(f'点云解析失败: {e}', throttle_duration_sec=2.0)

        return None

    # =========================================================================
    # 发布检测结果
    # =========================================================================
    def _publish_detections(
        self,
        detections: List[Detection3D],
        stamp,
        frame_id: str
    ):
        """
        发布检测结果到 /pallet_detections，
        并将最近的托盘发布到 /pallet_pose，
        同时在TF树中广播 pallet_0, pallet_1, ... 坐标系。
        """
        if not detections:
            return

        # ---- 发布 Detection3DArray ----
        array_msg = Detection3DArray()
        array_msg.header.stamp    = stamp
        array_msg.header.frame_id = frame_id
        array_msg.detections      = detections
        self._detection_pub.publish(array_msg)

        # ---- 找到最近的托盘 ----
        nearest = min(
            detections,
            key=lambda d: d.bbox.center.position.z  # 相机坐标系：Z为前方距离
        )

        # ---- 发布最近托盘位姿 ----
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = nearest.bbox.center.position.x
        pose_msg.pose.position.y = nearest.bbox.center.position.y
        pose_msg.pose.position.z = nearest.bbox.center.position.z
        pose_msg.pose.orientation.w = 1.0
        self._pallet_pose_pub.publish(pose_msg)

        # ---- 广播 TF：pallet_0, pallet_1, ... ----
        for i, det in enumerate(detections):
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = stamp
            tf_msg.header.frame_id = frame_id
            tf_msg.child_frame_id  = f'pallet_{i}'

            tf_msg.transform.translation.x = det.bbox.center.position.x
            tf_msg.transform.translation.y = det.bbox.center.position.y
            tf_msg.transform.translation.z = det.bbox.center.position.z
            tf_msg.transform.rotation      = det.bbox.center.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        # ---- 打印日志 ----
        self._detection_count += 1
        if self._detection_count % 10 == 0:  # 每10次打印一次
            dist_nearest = nearest.bbox.center.position.z
            conf_nearest = nearest.results[0].hypothesis.score if nearest.results else 0.0
            self.get_logger().info(
                f'检测到 {len(detections)} 个托盘 | '
                f'最近: {dist_nearest:.2f}m @ '
                f'({nearest.bbox.center.position.x:.2f}, '
                f'{nearest.bbox.center.position.y:.2f}, '
                f'{nearest.bbox.center.position.z:.2f}) | '
                f'置信度: {conf_nearest:.2f}'
            )


# =============================================================================
# Main
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PalletDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('pallet_detector 节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
