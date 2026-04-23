#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
货叉控制节点
============
实现叉车货叉的平滑运动控制，采用梯形速度曲线（加速→匀速→减速）。

订阅：
  /fork_position     (std_msgs/Float64) — 目标高度（0.0~1.0 归一化）
  /joint_states      (sensor_msgs/JointState) — 当前关节状态（货叉位置反馈）

发布：
  /fork_joint_controller/command  (std_msgs/Float64) — prismatic joint 控制指令（米）
  /fork_status                    (std_msgs/String)  — 状态: MOVING / REACHED / ERROR

ROS2 Humble 环境
"""

import time
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState


# ============================================================
# 梯形速度曲线规划器
# ============================================================
class TrapezoidalMotionPlanner:
    """
    梯形速度曲线运动规划器
    
    速度曲线：
      加速段：从 0 加速到 max_velocity（在 accel_distance 内完成）
      匀速段：以 max_velocity 匀速运行
      减速段：从 max_velocity 减速到 0（在 decel_distance 内完成）
    
    确保运动平滑，避免对机械结构的冲击。
    """

    def __init__(self,
                 max_velocity: float = 0.05,    # 最大速度（m/s）
                 acceleration: float = 0.02,     # 加速度（m/s²）
                 position_tolerance: float = 0.005  # 到位容差（米）
                 ):
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.position_tolerance = position_tolerance

        # 运动状态
        self._current_position = 0.0
        self._current_velocity = 0.0
        self._target_position = 0.0
        self._is_moving = False
        self._last_time = None

    def set_target(self, target: float):
        """设置新的目标位置"""
        if abs(target - self._target_position) > self.position_tolerance:
            self._target_position = target
            self._is_moving = True
            self._last_time = time.time()

    def update(self) -> float:
        """
        更新运动状态，返回当前指令位置
        
        采用梯形速度曲线：
          - 根据剩余距离判断是否需要减速
          - 加速或减速时限制速度变化率
        """
        if not self._is_moving:
            return self._current_position

        current_time = time.time()
        if self._last_time is None:
            self._last_time = current_time
            return self._current_position

        dt = current_time - self._last_time
        self._last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return self._current_position

        # 计算位置误差
        error = self._target_position - self._current_position
        distance = abs(error)

        # 判断运动方向
        direction = 1.0 if error > 0 else -1.0

        # 检查是否到位
        if distance <= self.position_tolerance:
            self._current_position = self._target_position
            self._current_velocity = 0.0
            self._is_moving = False
            return self._current_position

        # 计算减速所需距离: v²/(2a)
        decel_distance = (self._current_velocity ** 2) / (2.0 * self.acceleration + 1e-6)

        # 决策：是否需要减速
        if distance <= decel_distance + self.position_tolerance:
            # 减速段：降低速度
            target_velocity = math.sqrt(2.0 * self.acceleration * distance)
            target_velocity = min(target_velocity, self.max_velocity)
        else:
            # 加速段或匀速段：向最大速度靠近
            target_velocity = self.max_velocity

        # 限制加速度（速度变化不超过 a*dt）
        velocity_change = self.acceleration * dt
        if target_velocity > self._current_velocity:
            self._current_velocity = min(
                self._current_velocity + velocity_change,
                target_velocity
            )
        else:
            self._current_velocity = max(
                self._current_velocity - velocity_change,
                target_velocity
            )

        # 确保速度不为负
        self._current_velocity = max(0.0, self._current_velocity)

        # 更新位置
        self._current_position += direction * self._current_velocity * dt

        # 防止超过目标
        if direction > 0:
            self._current_position = min(self._current_position, self._target_position)
        else:
            self._current_position = max(self._current_position, self._target_position)

        return self._current_position

    @property
    def is_moving(self) -> bool:
        return self._is_moving

    @property
    def current_position(self) -> float:
        return self._current_position

    @property
    def current_velocity(self) -> float:
        return self._current_velocity

    def force_position(self, position: float):
        """强制设置当前位置（初始化用）"""
        self._current_position = position
        self._target_position = position
        self._current_velocity = 0.0
        self._is_moving = False


# ============================================================
# 货叉控制节点
# ============================================================
class ForkController(Node):
    """
    货叉控制节点
    
    接收归一化高度指令（0.0~1.0），将其映射为实际关节位置（米），
    并通过梯形速度曲线平滑驱动货叉关节。
    
    关节映射：
      归一化 0.0 → 实际最低位（fork_min_height）
      归一化 1.0 → 实际最高位（fork_max_height）
    """

    # ---------- 货叉关节参数（根据 URDF 调整）----------
    # 货叉最低高度（米，prismatic joint 最小值）
    FORK_MIN_HEIGHT = 0.0
    # 货叉最高高度（米，prismatic joint 最大值）
    FORK_MAX_HEIGHT = 1.5
    # 货叉关节名称（与 URDF/Gazebo 一致）
    FORK_JOINT_NAME = 'fork_lift_joint'
    # 控制发布频率（Hz）
    CONTROL_RATE = 50.0
    # 到位判断容差（米）
    REACHED_TOLERANCE = 0.01
    # 状态发布频率（Hz）
    STATUS_RATE = 10.0

    def __init__(self):
        super().__init__('fork_controller')

        # 声明并读取参数（允许外部配置）
        self.declare_parameter('fork_min_height', self.FORK_MIN_HEIGHT)
        self.declare_parameter('fork_max_height', self.FORK_MAX_HEIGHT)
        self.declare_parameter('fork_joint_name', self.FORK_JOINT_NAME)
        self.declare_parameter('max_velocity', 0.05)
        self.declare_parameter('acceleration', 0.02)

        self.FORK_MIN_HEIGHT = self.get_parameter('fork_min_height').value
        self.FORK_MAX_HEIGHT = self.get_parameter('fork_max_height').value
        self.FORK_JOINT_NAME = self.get_parameter('fork_joint_name').value
        max_vel = self.get_parameter('max_velocity').value
        accel = self.get_parameter('acceleration').value

        # ---------- 运动规划器 ----------
        self._planner = TrapezoidalMotionPlanner(
            max_velocity=max_vel,
            acceleration=accel,
            position_tolerance=self.REACHED_TOLERANCE
        )

        # ---------- 内部状态 ----------
        self._target_normalized = 0.0          # 目标高度（归一化）
        self._current_joint_pos = 0.0          # 当前关节位置（来自 joint_states）
        self._joint_feedback_available = False  # 是否有关节反馈
        self._status = 'REACHED'               # 当前状态: MOVING / REACHED / ERROR
        self._initialized = False              # 是否已初始化位置

        # 回调组
        self._cb_group = ReentrantCallbackGroup()

        # ---------- 订阅者 ----------
        # 接收目标高度（归一化 0~1）
        self._sub_fork_position = self.create_subscription(
            Float64,
            '/fork_position',
            self._cb_fork_position,
            10,
            callback_group=self._cb_group
        )
        # 接收关节状态反馈
        self._sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            10,
            callback_group=self._cb_group
        )

        # ---------- 发布者 ----------
        # 向 Gazebo 关节控制器发送指令
        self._pub_joint_cmd = self.create_publisher(
            Float64,
            '/fork_joint_controller/command',
            10
        )
        # 发布货叉状态
        self._pub_fork_status = self.create_publisher(
            String,
            '/fork_status',
            10
        )

        # ---------- 定时器 ----------
        # 控制循环（50Hz）
        self._control_timer = self.create_timer(
            1.0 / self.CONTROL_RATE,
            self._control_loop,
            callback_group=self._cb_group
        )
        # 状态发布（10Hz）
        self._status_timer = self.create_timer(
            1.0 / self.STATUS_RATE,
            self._publish_status,
            callback_group=self._cb_group
        )

        self.get_logger().info('='*40)
        self.get_logger().info('货叉控制节点已启动')
        self.get_logger().info(
            f'关节: {self.FORK_JOINT_NAME}, '
            f'高度范围: [{self.FORK_MIN_HEIGHT}, {self.FORK_MAX_HEIGHT}]m'
        )
        self.get_logger().info('='*40)

    # ============================================================
    # 订阅回调
    # ============================================================

    def _cb_fork_position(self, msg: Float64):
        """
        接收目标货叉高度（归一化 0.0~1.0）
        0.0 = 最低位（托盘插槽高度）
        1.0 = 最高位（抬起搬运高度）
        """
        normalized = max(0.0, min(1.0, msg.data))  # 限幅

        if abs(normalized - self._target_normalized) > 0.001:
            self._target_normalized = normalized
            actual_height = self._normalized_to_actual(normalized)
            self._planner.set_target(actual_height)
            self.get_logger().info(
                f'收到货叉高度指令: {normalized:.2f} → 实际 {actual_height:.3f}m'
            )

    def _cb_joint_states(self, msg: JointState):
        """接收关节状态，提取货叉关节位置"""
        try:
            idx = msg.name.index(self.FORK_JOINT_NAME)
            self._current_joint_pos = msg.position[idx]
            self._joint_feedback_available = True

            # 首次收到关节反馈时，初始化规划器位置
            if not self._initialized:
                self._planner.force_position(self._current_joint_pos)
                self._initialized = True
                self.get_logger().info(
                    f'初始化货叉位置: {self._current_joint_pos:.3f}m'
                )
        except ValueError:
            # 关节名不在列表中，静默处理
            pass

    # ============================================================
    # 控制循环
    # ============================================================

    def _control_loop(self):
        """
        50Hz 控制循环
        更新运动规划器并发布关节指令
        """
        if not self._initialized:
            # 未初始化时发送零指令保持静止
            self._publish_joint_command(0.0)
            return

        # 更新梯形速度规划
        cmd_position = self._planner.update()

        # 安全限幅
        cmd_position = max(self.FORK_MIN_HEIGHT,
                          min(self.FORK_MAX_HEIGHT, cmd_position))

        # 发布关节控制指令
        self._publish_joint_command(cmd_position)

        # 更新状态
        self._update_status(cmd_position)

    def _update_status(self, cmd_position: float):
        """根据运动状态更新状态字符串"""
        if self._planner.is_moving:
            self._status = 'MOVING'
        else:
            # 检查是否实际到达目标位置（使用关节反馈）
            if self._joint_feedback_available:
                actual_error = abs(self._current_joint_pos - cmd_position)
                if actual_error < self.REACHED_TOLERANCE * 2:
                    self._status = 'REACHED'
                else:
                    self._status = 'MOVING'
            else:
                self._status = 'REACHED'

    # ============================================================
    # 发布函数
    # ============================================================

    def _publish_joint_command(self, position: float):
        """向 Gazebo 关节控制器发送位置指令"""
        msg = Float64()
        msg.data = position
        self._pub_joint_cmd.publish(msg)

    def _publish_status(self):
        """发布货叉当前状态字符串"""
        msg = String()
        msg.data = self._status
        self._pub_fork_status.publish(msg)

        # 调试日志（低频）
        self.get_logger().debug(
            f'货叉状态: {self._status}, '
            f'目标: {self._planner._target_position:.3f}m, '
            f'当前: {self._planner.current_position:.3f}m, '
            f'速度: {self._planner.current_velocity:.4f}m/s'
        )

    # ============================================================
    # 工具函数
    # ============================================================

    def _normalized_to_actual(self, normalized: float) -> float:
        """归一化高度转实际高度（米）"""
        return (self.FORK_MIN_HEIGHT +
                normalized * (self.FORK_MAX_HEIGHT - self.FORK_MIN_HEIGHT))

    def _actual_to_normalized(self, actual: float) -> float:
        """实际高度转归一化高度"""
        height_range = self.FORK_MAX_HEIGHT - self.FORK_MIN_HEIGHT
        if height_range <= 0:
            return 0.0
        return (actual - self.FORK_MIN_HEIGHT) / height_range


# ============================================================
# 主入口
# ============================================================
def main(args=None):
    """货叉控制节点主入口"""
    rclpy.init(args=args)

    node = ForkController()

    try:
        node.get_logger().info('货叉控制节点正在运行...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，节点关闭')
    finally:
        # 降叉归位（安全停止）
        node._publish_joint_command(node.FORK_MIN_HEIGHT)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
