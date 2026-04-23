#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
叉车任务状态机节点
==================
实现叉车自动化货物搬运的完整状态机流程：
IDLE → NAVIGATE_TO_PALLET → DETECT_PALLET → ALIGN_TO_PALLET →
LOWER_FORK → PICKUP_PALLET → NAVIGATE_TO_DESTINATION → DEPOSIT_PALLET → DONE

ROS2 Humble 环境
作者: Forklift Simulation Team
"""

import math
import time
import enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

# 消息类型导入
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Nav2 Action 类型
from nav2_msgs.action import NavigateToPose

# ROS2 服务类型
from std_srvs.srv import Trigger

# TF2 相关
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# 角度工具
from geometry_msgs.msg import Quaternion


# ============================================================
# 状态枚举定义
# ============================================================
class TaskState(enum.Enum):
    """叉车任务状态枚举"""
    IDLE = "IDLE"                                   # 空闲等待
    NAVIGATE_TO_PALLET = "NAVIGATE_TO_PALLET"       # 导航到托盘附近
    DETECT_PALLET = "DETECT_PALLET"                 # 视觉检测托盘
    ALIGN_TO_PALLET = "ALIGN_TO_PALLET"             # 精细对准货叉
    LOWER_FORK = "LOWER_FORK"                       # 降低货叉
    PICKUP_PALLET = "PICKUP_PALLET"                 # 抬起货物
    NAVIGATE_TO_DESTINATION = "NAVIGATE_TO_DESTINATION"  # 导航到目的地
    DEPOSIT_PALLET = "DEPOSIT_PALLET"               # 放置货物
    DONE = "DONE"                                   # 任务完成
    ERROR = "ERROR"                                 # 错误状态


# ============================================================
# PID 控制器（用于对准阶段）
# ============================================================
class PIDController:
    """
    简单 PID 控制器
    用于货叉对准时的角度和距离控制
    """
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def compute(self, error: float) -> float:
        """计算 PID 输出值"""
        current_time = time.time()

        if self._last_time is None:
            dt = 0.1  # 默认时间步长
        else:
            dt = current_time - self._last_time
            if dt <= 0.0:
                dt = 0.001

        # 积分项（带饱和保护）
        self._integral += error * dt
        self._integral = max(-10.0, min(10.0, self._integral))

        # 微分项
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0

        # PID 输出
        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # 限幅
        output = max(self.output_min, min(self.output_max, output))

        self._last_error = error
        self._last_time = current_time
        return output

    def reset(self):
        """重置 PID 状态"""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None


# ============================================================
# 工具函数
# ============================================================
def yaw_from_quaternion(q: Quaternion) -> float:
    """从四元数提取偏航角（yaw）"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """欧拉角转四元数"""
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def distance_2d(pose1: Pose, pose2: Pose) -> float:
    """计算两个位姿之间的2D欧氏距离"""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx * dx + dy * dy)


# ============================================================
# 任务状态机主节点
# ============================================================
class ForkliftTaskManager(Node):
    """
    叉车任务管理器
    
    核心状态机节点，协调叉车完成托盘搬运的全流程：
    感知托盘 → 导航靠近 → 视觉对准 → 货叉插入 → 搬运 → 放置
    """

    # ---------- 配置参数 ----------
    # 导航接近距离：在离托盘多远的地方停止（米）
    PALLET_APPROACH_DISTANCE = 1.5
    # 对准目标距离：货叉尖端距托盘插槽中心的目标距离（米）
    ALIGN_TARGET_DISTANCE = 0.3
    # 对准角度容差（弧度）
    ALIGN_ANGLE_TOLERANCE = 0.05
    # 对准距离容差（米）
    ALIGN_DIST_TOLERANCE = 0.05
    # 检测超时时间（秒）
    DETECT_TIMEOUT = 30.0
    # 对准超时时间（秒）
    ALIGN_TIMEOUT = 60.0
    # 货叉降低等待时间（秒）
    FORK_LOWER_WAIT = 2.0
    # 取货前进速度（米/秒）
    PICKUP_LINEAR_SPEED = 0.1
    # 取货前进距离（米）
    PICKUP_DISTANCE = 0.8
    # 目的地坐标（仓库出口，可按实际修改）
    DESTINATION_X = 10.0
    DESTINATION_Y = 0.0
    DESTINATION_YAW = 0.0
    # 放置后退速度（米/秒）
    DEPOSIT_REVERSE_SPEED = -0.15
    # 放置后退距离（米）
    DEPOSIT_DISTANCE = 1.0
    # 货叉插入后的升叉高度（归一化 0~1）
    FORK_PICKUP_HEIGHT = 0.2
    # 控制发布频率（Hz）
    CONTROL_RATE = 20.0

    def __init__(self):
        super().__init__('forklift_task_manager')

        # ---------- 回调组（允许并发订阅）----------
        self._cb_group_sub = ReentrantCallbackGroup()
        self._cb_group_action = MutuallyExclusiveCallbackGroup()
        self._cb_group_srv = MutuallyExclusiveCallbackGroup()

        # ---------- 内部状态变量 ----------
        self._current_state = TaskState.IDLE
        self._previous_state = None

        # 托盘列表（来自 /spawned_pallets）
        self._pallet_list: list = []
        # 目标托盘位姿
        self._target_pallet_pose: Optional[Pose] = None
        # 检测到的托盘位姿（来自视觉）
        self._detected_pallet_pose: Optional[PoseStamped] = None
        self._pallet_detected_time: Optional[float] = None
        # 叉车当前位姿
        self._current_pose: Optional[Pose] = None
        # 导航状态
        self._nav_status: str = ""
        # 当前关节状态（货叉位置）
        self._joint_states: Optional[JointState] = None

        # 状态进入时间（用于超时检测）
        self._state_enter_time: float = time.time()
        # 取货阶段行驶计数
        self._pickup_distance_traveled = 0.0
        self._pickup_start_pose: Optional[Pose] = None
        # 放置阶段行驶计数
        self._deposit_distance_traveled = 0.0
        self._deposit_start_pose: Optional[Pose] = None

        # 对准 PID 控制器
        #   angular_pid: 控制转向（角度误差 → 角速度）
        #   linear_pid:  控制前后（距离误差 → 线速度）
        self._angular_pid = PIDController(kp=1.2, ki=0.01, kd=0.1,
                                          output_min=-0.5, output_max=0.5)
        self._linear_pid = PIDController(kp=0.8, ki=0.005, kd=0.05,
                                         output_min=-0.3, output_max=0.3)

        # TF2 缓冲区（用于坐标变换）
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Nav2 Action 客户端
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group_action
        )
        self._nav_goal_handle = None
        self._nav_result_future = None

        # ---------- 订阅者 ----------
        # 托盘生成列表
        self._sub_pallets = self.create_subscription(
            PoseArray,
            '/spawned_pallets',
            self._cb_spawned_pallets,
            10,
            callback_group=self._cb_group_sub
        )
        # 视觉检测到的托盘位姿
        self._sub_pallet_pose = self.create_subscription(
            PoseStamped,
            '/pallet_pose',
            self._cb_pallet_pose,
            10,
            callback_group=self._cb_group_sub
        )
        # 导航状态反馈
        self._sub_nav_status = self.create_subscription(
            String,
            '/nav_status',
            self._cb_nav_status,
            10,
            callback_group=self._cb_group_sub
        )
        # 里程计（获取当前位姿）
        self._sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._cb_odom,
            10,
            callback_group=self._cb_group_sub
        )
        # 关节状态（可选，用于货叉位置反馈）
        self._sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            10,
            callback_group=self._cb_group_sub
        )

        # ---------- 发布者 ----------
        # 运动控制指令
        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # 货叉位置控制（归一化 0~1）
        self._pub_fork_position = self.create_publisher(Float64, '/fork_position', 10)
        # 当前任务状态字符串
        self._pub_task_status = self.create_publisher(String, '/task_status', 10)

        # ---------- 服务服务器 ----------
        # 触发取货任务
        self._srv_start_task = self.create_service(
            Trigger,
            '/start_pickup_task',
            self._cb_start_task,
            callback_group=self._cb_group_srv
        )

        # ---------- 状态机主循环定时器 ----------
        self._timer = self.create_timer(
            1.0 / self.CONTROL_RATE,
            self._state_machine_tick,
            callback_group=self._cb_group_sub
        )

        # ---------- 状态发布定时器（低频）----------
        self._status_timer = self.create_timer(
            1.0,
            self._publish_status,
            callback_group=self._cb_group_sub
        )

        self.get_logger().info('=' * 50)
        self.get_logger().info('叉车任务管理器已启动')
        self.get_logger().info('调用 /start_pickup_task 服务以触发任务')
        self.get_logger().info('=' * 50)

    # ============================================================
    # 订阅回调函数
    # ============================================================

    def _cb_spawned_pallets(self, msg: PoseArray):
        """接收托盘列表"""
        self._pallet_list = list(msg.poses)
        self.get_logger().debug(f'收到托盘列表，共 {len(self._pallet_list)} 个托盘')

    def _cb_pallet_pose(self, msg: PoseStamped):
        """接收视觉检测到的托盘位姿"""
        self._detected_pallet_pose = msg
        self._pallet_detected_time = time.time()
        self.get_logger().debug(
            f'检测到托盘: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}'
        )

    def _cb_nav_status(self, msg: String):
        """接收导航状态"""
        self._nav_status = msg.data

    def _cb_odom(self, msg: Odometry):
        """接收里程计，更新当前位姿"""
        self._current_pose = msg.pose.pose

    def _cb_joint_states(self, msg: JointState):
        """接收关节状态（货叉高度等）"""
        self._joint_states = msg

    # ============================================================
    # 服务回调
    # ============================================================

    def _cb_start_task(self, request, response):
        """
        处理 /start_pickup_task 服务请求
        只有在 IDLE 或 DONE 状态下才能触发新任务
        """
        if self._current_state in (TaskState.IDLE, TaskState.DONE):
            if len(self._pallet_list) == 0:
                response.success = False
                response.message = '没有可用的托盘（/spawned_pallets 尚未收到数据）'
                self.get_logger().warn(response.message)
                return response

            self.get_logger().info('收到任务触发请求，开始执行搬运任务')
            self._transition_to(TaskState.NAVIGATE_TO_PALLET)
            response.success = True
            response.message = f'任务已启动，当前有 {len(self._pallet_list)} 个托盘'
        else:
            response.success = False
            response.message = (f'当前处于 {self._current_state.value} 状态，'
                                f'无法启动新任务')
            self.get_logger().warn(response.message)
        return response

    # ============================================================
    # 状态机核心
    # ============================================================

    def _transition_to(self, new_state: TaskState):
        """执行状态转换，记录日志并重置计时器"""
        old_state = self._current_state
        self._previous_state = old_state
        self._current_state = new_state
        self._state_enter_time = time.time()

        self.get_logger().info(
            f'[状态转换] {old_state.value} → {new_state.value}'
        )

        # 进入新状态时的初始化动作
        self._on_state_enter(new_state)

    def _on_state_enter(self, state: TaskState):
        """进入新状态时执行一次性初始化动作"""
        if state == TaskState.NAVIGATE_TO_PALLET:
            # 选择最近的托盘作为目标
            self._select_nearest_pallet()

        elif state == TaskState.DETECT_PALLET:
            # 重置检测数据
            self._detected_pallet_pose = None
            self._pallet_detected_time = None
            self.get_logger().info('开始等待视觉检测数据，超时 30s')

        elif state == TaskState.ALIGN_TO_PALLET:
            # 重置 PID 控制器
            self._angular_pid.reset()
            self._linear_pid.reset()
            self.get_logger().info('开始精细对准，目标距离 0.3m')

        elif state == TaskState.LOWER_FORK:
            # 发送降叉指令
            self._publish_fork_position(0.0)
            self.get_logger().info('发送降叉指令（fork_position=0.0），等待 2s')

        elif state == TaskState.PICKUP_PALLET:
            # 记录起始位置，用于计算行驶距离
            self._pickup_start_pose = self._current_pose
            self._pickup_distance_traveled = 0.0
            self.get_logger().info('开始取货：缓慢前进插入货叉')

        elif state == TaskState.NAVIGATE_TO_DESTINATION:
            self.get_logger().info(
                f'导航到目的地: ({self.DESTINATION_X}, {self.DESTINATION_Y})'
            )
            self._send_navigation_goal(
                self.DESTINATION_X,
                self.DESTINATION_Y,
                self.DESTINATION_YAW
            )

        elif state == TaskState.DEPOSIT_PALLET:
            # 先降叉
            self._publish_fork_position(0.0)
            self._deposit_start_pose = self._current_pose
            self._deposit_distance_traveled = 0.0
            self.get_logger().info('开始放置货物：降叉并后退')

        elif state == TaskState.DONE:
            # 停止运动
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info('='*40)
            self.get_logger().info('任务完成！叉车进入 DONE 状态')
            self.get_logger().info('='*40)

        elif state == TaskState.ERROR:
            # 紧急停止
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().error('任务进入 ERROR 状态，叉车已停止')

    def _state_machine_tick(self):
        """状态机主循环（20Hz 定时调用）"""
        state = self._current_state

        if state == TaskState.IDLE:
            self._tick_idle()
        elif state == TaskState.NAVIGATE_TO_PALLET:
            self._tick_navigate_to_pallet()
        elif state == TaskState.DETECT_PALLET:
            self._tick_detect_pallet()
        elif state == TaskState.ALIGN_TO_PALLET:
            self._tick_align_to_pallet()
        elif state == TaskState.LOWER_FORK:
            self._tick_lower_fork()
        elif state == TaskState.PICKUP_PALLET:
            self._tick_pickup_pallet()
        elif state == TaskState.NAVIGATE_TO_DESTINATION:
            self._tick_navigate_to_destination()
        elif state == TaskState.DEPOSIT_PALLET:
            self._tick_deposit_pallet()
        elif state == TaskState.DONE:
            pass  # 等待下一任务指令
        elif state == TaskState.ERROR:
            pass  # 等待人工干预

    # ============================================================
    # 各状态 tick 函数
    # ============================================================

    def _tick_idle(self):
        """IDLE 状态：等待任务触发，无动作"""
        pass

    def _tick_navigate_to_pallet(self):
        """
        NAVIGATE_TO_PALLET 状态：
        使用 Nav2 导航到目标托盘附近（1.5m 停止点）
        """
        if self._target_pallet_pose is None:
            self.get_logger().warn('没有目标托盘，返回 IDLE')
            self._transition_to(TaskState.IDLE)
            return

        if self._current_pose is None:
            return  # 等待里程计数据

        # 检查是否已接近目标托盘
        dist = distance_2d(self._current_pose, self._target_pallet_pose)

        if dist <= self.PALLET_APPROACH_DISTANCE:
            self.get_logger().info(
                f'已到达托盘附近（距离 {dist:.2f}m），切换到检测状态'
            )
            # 停止导航
            self._cancel_navigation()
            self._publish_cmd_vel(0.0, 0.0)
            self._transition_to(TaskState.DETECT_PALLET)
            return

        # 检查是否有导航目标（如果还未发送，则发送）
        if self._nav_goal_handle is None:
            # 计算接近点：在托盘前方 PALLET_APPROACH_DISTANCE 处
            approach_x, approach_y = self._compute_approach_point(
                self._target_pallet_pose,
                self.PALLET_APPROACH_DISTANCE
            )
            # 朝向托盘的偏航角
            dx = self._target_pallet_pose.position.x - approach_x
            dy = self._target_pallet_pose.position.y - approach_y
            yaw = math.atan2(dy, dx)
            self._send_navigation_goal(approach_x, approach_y, yaw)

    def _tick_detect_pallet(self):
        """
        DETECT_PALLET 状态：
        等待视觉检测节点发布有效的托盘位姿
        超时 30s 则进入错误状态
        """
        elapsed = time.time() - self._state_enter_time

        # 检查超时
        if elapsed > self.DETECT_TIMEOUT:
            self.get_logger().error(
                f'托盘检测超时（{self.DETECT_TIMEOUT}s），任务失败'
            )
            self._transition_to(TaskState.ERROR)
            return

        # 检查是否收到有效检测数据（1s 内的新数据）
        if (self._detected_pallet_pose is not None and
                self._pallet_detected_time is not None and
                time.time() - self._pallet_detected_time < 1.0):
            self.get_logger().info(
                f'托盘检测成功！位置: x={self._detected_pallet_pose.pose.position.x:.3f}'
            )
            self._transition_to(TaskState.ALIGN_TO_PALLET)
        else:
            # 每 5 秒打印一次等待日志
            if int(elapsed) % 5 == 0 and elapsed > 0.5:
                self.get_logger().info(
                    f'等待托盘检测数据... ({elapsed:.0f}s / {self.DETECT_TIMEOUT}s)'
                )

    def _tick_align_to_pallet(self):
        """
        ALIGN_TO_PALLET 状态：
        使用 PID 控制精细对准货叉与托盘插槽
        
        控制策略：
        1. 角度控制：转动叉车使货叉正对托盘插槽
        2. 距离控制：前后调整到目标距离 0.3m
        """
        # 超时保护
        elapsed = time.time() - self._state_enter_time
        if elapsed > self.ALIGN_TIMEOUT:
            self.get_logger().error(f'对准超时（{self.ALIGN_TIMEOUT}s）')
            self._publish_cmd_vel(0.0, 0.0)
            self._transition_to(TaskState.ERROR)
            return

        # 检查视觉数据是否仍有效（2s 内）
        if (self._detected_pallet_pose is None or
                self._pallet_detected_time is None or
                time.time() - self._pallet_detected_time > 2.0):
            self.get_logger().warn('视觉数据丢失，重新进入检测状态')
            self._publish_cmd_vel(0.0, 0.0)
            self._transition_to(TaskState.DETECT_PALLET)
            return

        # 从视觉数据获取托盘相对于叉车的位置
        # /pallet_pose 通常发布在 camera_frame 坐标系下
        pallet = self._detected_pallet_pose.pose

        # 计算相对角度和距离
        # 假设 pallet_pose 已经转换到叉车基坐标系 (base_link)
        # x 轴正方向为叉车前进方向
        pallet_x = pallet.position.x  # 纵向距离
        pallet_y = pallet.position.y  # 横向偏移

        # 角度误差：托盘在叉车坐标系中的横向偏移对应的角度
        angle_error = math.atan2(pallet_y, pallet_x)
        # 距离误差：当前距离与目标距离的差
        current_dist = math.sqrt(pallet_x**2 + pallet_y**2)
        dist_error = current_dist - self.ALIGN_TARGET_DISTANCE

        # 检查是否已对准
        if (abs(angle_error) < self.ALIGN_ANGLE_TOLERANCE and
                abs(dist_error) < self.ALIGN_DIST_TOLERANCE):
            self.get_logger().info(
                f'对准完成！角度误差: {math.degrees(angle_error):.2f}°, '
                f'距离误差: {dist_error:.3f}m'
            )
            self._publish_cmd_vel(0.0, 0.0)
            self._transition_to(TaskState.LOWER_FORK)
            return

        # PID 计算控制量
        angular_vel = self._angular_pid.compute(-angle_error)
        linear_vel = self._linear_pid.compute(dist_error)

        # 如果角度误差较大，优先转向（减小线速度）
        if abs(angle_error) > 0.2:
            linear_vel *= 0.3

        self._publish_cmd_vel(linear_vel, angular_vel)

        # 周期性日志（每秒一次）
        if int(elapsed * 2) % 10 == 0:
            self.get_logger().info(
                f'对准中: 角度误差={math.degrees(angle_error):.1f}°, '
                f'距离误差={dist_error:.3f}m, '
                f'线速={linear_vel:.3f}, 角速={angular_vel:.3f}'
            )

    def _tick_lower_fork(self):
        """
        LOWER_FORK 状态：
        等待货叉降低到位（简单等待 2s）
        """
        elapsed = time.time() - self._state_enter_time

        if elapsed >= self.FORK_LOWER_WAIT:
            self.get_logger().info('货叉已降低，准备插入')
            self._transition_to(TaskState.PICKUP_PALLET)

    def _tick_pickup_pallet(self):
        """
        PICKUP_PALLET 状态：
        1. 缓慢前进 0.8m，将货叉插入托盘插槽
        2. 然后升叉抬起货物
        """
        if self._current_pose is None:
            return

        # 计算已行驶距离
        if self._pickup_start_pose is not None:
            self._pickup_distance_traveled = distance_2d(
                self._current_pose,
                self._pickup_start_pose
            )

        # 阶段一：前进插入货叉
        if self._pickup_distance_traveled < self.PICKUP_DISTANCE:
            self._publish_cmd_vel(self.PICKUP_LINEAR_SPEED, 0.0)
            self.get_logger().debug(
                f'前进中: {self._pickup_distance_traveled:.3f}m '
                f'/ {self.PICKUP_DISTANCE}m'
            )
        else:
            # 阶段二：停止前进，升叉
            self._publish_cmd_vel(0.0, 0.0)

            # 检查是否刚完成前进（一次性动作）
            if not hasattr(self, '_fork_raised') or not self._fork_raised:
                self.get_logger().info('货叉已插入托盘，开始升叉')
                self._publish_fork_position(self.FORK_PICKUP_HEIGHT)
                self._fork_raise_time = time.time()
                self._fork_raised = True

            # 等待货叉升起（1.5s）
            if hasattr(self, '_fork_raise_time'):
                if time.time() - self._fork_raise_time >= 1.5:
                    self._fork_raised = False  # 重置标志
                    self.get_logger().info('货物已抬起，开始导航到目的地')
                    self._transition_to(TaskState.NAVIGATE_TO_DESTINATION)

    def _tick_navigate_to_destination(self):
        """
        NAVIGATE_TO_DESTINATION 状态：
        导航到预设目的地（仓库出口）
        等待 Nav2 完成导航
        """
        # 检查是否到达目的地（通过当前位置判断）
        if self._current_pose is not None:
            target = Pose()
            target.position.x = self.DESTINATION_X
            target.position.y = self.DESTINATION_Y

            dist = distance_2d(self._current_pose, target)
            if dist < 0.5:  # 0.5m 内视为到达
                self.get_logger().info(
                    f'已到达目的地（距离 {dist:.2f}m），开始放置货物'
                )
                self._cancel_navigation()
                self._transition_to(TaskState.DEPOSIT_PALLET)
                return

        # 检查导航超时（120s）
        elapsed = time.time() - self._state_enter_time
        if elapsed > 120.0:
            self.get_logger().error('导航到目的地超时（120s）')
            self._transition_to(TaskState.ERROR)

    def _tick_deposit_pallet(self):
        """
        DEPOSIT_PALLET 状态：
        1. 降叉放置货物（货叉已在降低位置）
        2. 后退脱出托盘
        """
        if self._current_pose is None:
            return

        elapsed = time.time() - self._state_enter_time

        # 等待货叉完全降下（2s）
        if elapsed < 2.0:
            return

        # 计算后退距离
        if self._deposit_start_pose is not None:
            self._deposit_distance_traveled = distance_2d(
                self._current_pose,
                self._deposit_start_pose
            )

        # 后退脱出
        if self._deposit_distance_traveled < self.DEPOSIT_DISTANCE:
            self._publish_cmd_vel(self.DEPOSIT_REVERSE_SPEED, 0.0)
            self.get_logger().debug(
                f'后退中: {self._deposit_distance_traveled:.3f}m '
                f'/ {self.DEPOSIT_DISTANCE}m'
            )
        else:
            # 放置完成
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info('货物放置完成，任务结束')
            self._transition_to(TaskState.DONE)

    # ============================================================
    # 辅助方法
    # ============================================================

    def _select_nearest_pallet(self):
        """从托盘列表中选择距叉车最近的托盘"""
        if not self._pallet_list:
            self.get_logger().warn('托盘列表为空')
            self._target_pallet_pose = None
            return

        if self._current_pose is None:
            # 无位置信息时选第一个托盘
            self._target_pallet_pose = self._pallet_list[0]
            self.get_logger().info('无位置信息，选择第一个托盘')
            return

        min_dist = float('inf')
        nearest = None

        for pallet_pose in self._pallet_list:
            d = distance_2d(self._current_pose, pallet_pose)
            if d < min_dist:
                min_dist = d
                nearest = pallet_pose

        self._target_pallet_pose = nearest
        self.get_logger().info(
            f'选择最近托盘: x={nearest.position.x:.2f}, '
            f'y={nearest.position.y:.2f}, 距离={min_dist:.2f}m'
        )

    def _compute_approach_point(self, pallet_pose: Pose, offset: float):
        """
        计算托盘前方的接近点坐标
        假设叉车需要从托盘正前方接近
        """
        # 获取托盘的朝向（偏航角）
        pallet_yaw = yaw_from_quaternion(pallet_pose.orientation)

        # 接近点 = 托盘位置 - offset * 朝向向量
        approach_x = pallet_pose.position.x - offset * math.cos(pallet_yaw)
        approach_y = pallet_pose.position.y - offset * math.sin(pallet_yaw)

        return approach_x, approach_y

    def _send_navigation_goal(self, x: float, y: float, yaw: float):
        """向 Nav2 发送导航目标"""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server 不可用')
            self._transition_to(TaskState.ERROR)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)

        self.get_logger().info(f'发送导航目标: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)')

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """Nav2 目标接受/拒绝回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝了导航目标')
            self._transition_to(TaskState.ERROR)
            return

        self._nav_goal_handle = goal_handle
        self.get_logger().info('Nav2 已接受导航目标')
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """Nav2 导航结果回调"""
        result = future.result()
        status = result.status

        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 导航成功')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Nav2 导航中止')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Nav2 导航被取消')

        self._nav_goal_handle = None

    def _nav_feedback_callback(self, feedback_msg):
        """Nav2 导航反馈（距离目标的剩余距离）"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'导航剩余距离: {feedback.distance_remaining:.2f}m'
        )

    def _cancel_navigation(self):
        """取消当前正在执行的导航任务"""
        if self._nav_goal_handle is not None:
            self.get_logger().info('取消当前导航任务')
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def _publish_cmd_vel(self, linear: float, angular: float):
        """发布运动控制指令"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub_cmd_vel.publish(msg)

    def _publish_fork_position(self, position: float):
        """
        发布货叉高度指令
        position: 0.0 = 最低位，1.0 = 最高位（归一化）
        """
        position = max(0.0, min(1.0, position))  # 限幅
        msg = Float64()
        msg.data = position
        self._pub_fork_position.publish(msg)
        self.get_logger().info(f'发布货叉高度指令: {position:.2f}')

    def _publish_status(self):
        """发布当前任务状态字符串（1Hz）"""
        msg = String()
        msg.data = self._current_state.value
        self._pub_task_status.publish(msg)


# ============================================================
# 主入口
# ============================================================
def main(args=None):
    """节点主入口"""
    rclpy.init(args=args)

    # 使用多线程执行器，允许 action client 与状态机并发运行
    executor = MultiThreadedExecutor(num_threads=4)

    node = ForkliftTaskManager()
    executor.add_node(node)

    try:
        node.get_logger().info('叉车任务管理器正在运行...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，节点关闭')
    finally:
        # 停止叉车
        stop_msg = Twist()
        node._pub_cmd_vel.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
