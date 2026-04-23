#!/usr/bin/env python3
# =============================================================================
# navigate_to_point.py
# 叉车导航控制节点
# 功能：
#   1. 订阅 /goal_point (geometry_msgs/PoseStamped) 接收目标点
#   2. 调用 Nav2 NavigateToPose action 执行导航
#   3. 发布导航状态 /nav_status (std_msgs/String)
#   4. 提供 /cancel_navigation service 取消当前导航
#   5. 打印导航进度日志
# 作者: forklift_dev
# =============================================================================

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# =============================================================================
# 导航状态常量
# =============================================================================
class NavStatus:
    IDLE        = "IDLE"        # 空闲，等待目标
    NAVIGATING  = "NAVIGATING"  # 正在导航
    REACHED     = "REACHED"     # 已到达目标
    FAILED      = "FAILED"      # 导航失败
    CANCELLED   = "CANCELLED"   # 已取消


# =============================================================================
# NavigateToPointNode
# =============================================================================
class NavigateToPointNode(Node):
    """
    叉车导航控制节点。
    将外部发布的目标点转发给 Nav2 NavigateToPose action，
    并反馈导航状态。
    """

    def __init__(self):
        super().__init__('navigate_to_point')

        # ---- 参数声明 ----
        self.declare_parameter('goal_tolerance_xy',  0.25)    # 到达目标的XY容差(m)
        self.declare_parameter('goal_tolerance_yaw', 0.25)    # 偏航角容差(rad)
        self.declare_parameter('navigate_action',    'navigate_to_pose')  # action名称
        self.declare_parameter('status_pub_rate',    2.0)     # 状态发布频率(Hz)

        # 读取参数
        self.goal_tolerance_xy  = self.get_parameter('goal_tolerance_xy').value
        self.goal_tolerance_yaw = self.get_parameter('goal_tolerance_yaw').value
        nav_action              = self.get_parameter('navigate_action').value
        status_pub_rate         = self.get_parameter('status_pub_rate').value

        # ---- 内部状态 ----
        self._current_status     = NavStatus.IDLE   # 当前导航状态
        self._goal_handle        = None             # 当前 action goal handle
        self._goal_lock          = threading.Lock() # 保护 goal handle 的锁
        self._current_goal_pose  = None             # 当前目标位姿
        self._feedback_distance  = 0.0              # 距目标的剩余距离

        # ---- 使用可重入回调组（允许 action + subscription 并发）----
        self._cb_group = ReentrantCallbackGroup()

        # ---- Action Client: NavigateToPose ----
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            nav_action,
            callback_group=self._cb_group
        )
        self.get_logger().info(f'等待 NavigateToPose action server ({nav_action})...')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateToPose action server 不可用，将在收到目标时重试')

        # ---- 订阅：/goal_point ----
        # 使用 TRANSIENT_LOCAL 保证最后发布的目标点被接收
        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_point',
            self._goal_point_callback,
            qos_profile=goal_qos,
            callback_group=self._cb_group
        )

        # ---- 发布：/nav_status ----
        self._status_pub = self.create_publisher(
            String,
            '/nav_status',
            10
        )

        # ---- Service: /cancel_navigation ----
        self._cancel_srv = self.create_service(
            Trigger,
            '/cancel_navigation',
            self._cancel_navigation_callback,
            callback_group=self._cb_group
        )

        # ---- 定时器：定期发布状态 ----
        self._status_timer = self.create_timer(
            1.0 / status_pub_rate,
            self._publish_status
        )

        self.get_logger().info('navigate_to_point 节点已启动')
        self.get_logger().info(f'  订阅目标点: /goal_point')
        self.get_logger().info(f'  状态发布:   /nav_status')
        self.get_logger().info(f'  取消服务:   /cancel_navigation')
        self.get_logger().info(f'  XY容差: {self.goal_tolerance_xy}m, Yaw容差: {self.goal_tolerance_yaw}rad')

    # =========================================================================
    # 回调：接收目标点
    # =========================================================================
    def _goal_point_callback(self, msg: PoseStamped):
        """
        接收外部发布的目标位姿，发送给 Nav2。
        如果当前正在导航，先取消再执行新目标。
        """
        x   = msg.pose.position.x
        y   = msg.pose.position.y
        yaw = self._quat_to_yaw(msg.pose.orientation)

        self.get_logger().info(
            f'收到新目标点: x={x:.3f}m, y={y:.3f}m, yaw={math.degrees(yaw):.1f}°'
        )

        # 如果正在导航，先取消
        if self._current_status == NavStatus.NAVIGATING:
            self.get_logger().info('取消当前导航，执行新目标...')
            self._cancel_current_goal()

        # 发送新目标给 Nav2
        self._send_goal(msg)

    # =========================================================================
    # 发送目标给 Nav2
    # =========================================================================
    def _send_goal(self, pose_stamped: PoseStamped):
        """
        通过 NavigateToPose action 发送目标到 Nav2。
        """
        # 确保 action server 可用
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server 不可用，导航失败')
            self._set_status(NavStatus.FAILED)
            return

        # 构造 action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        # 确保 frame_id 已设置
        if not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._current_goal_pose = pose_stamped
        self._set_status(NavStatus.NAVIGATING)

        self.get_logger().info(
            f'发送导航目标: ({goal_msg.pose.pose.position.x:.3f}, '
            f'{goal_msg.pose.pose.position.y:.3f}), '
            f'frame={goal_msg.pose.header.frame_id}'
        )

        # 异步发送 goal
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    # =========================================================================
    # Action 回调：goal 被接受/拒绝
    # =========================================================================
    def _goal_response_callback(self, future):
        """Nav2 接受或拒绝目标的回调"""
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝了导航目标！请检查目标点是否在地图范围内')
            self._set_status(NavStatus.FAILED)
            return

        self.get_logger().info('Nav2 已接受导航目标，开始规划路径...')

        with self._goal_lock:
            self._goal_handle = goal_handle

        # 等待导航结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    # =========================================================================
    # Action 回调：导航进度反馈
    # =========================================================================
    def _feedback_callback(self, feedback_msg):
        """
        Nav2 反馈回调：打印导航进度。
        NavigateToPose feedback 包含：
          - current_pose: 当前位姿
          - distance_remaining: 剩余距离
          - number_of_recoveries: 恢复尝试次数
          - navigation_time: 已用时间
        """
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        recoveries = feedback.number_of_recoveries
        nav_time = feedback.navigation_time.sec

        self._feedback_distance = dist

        # 每隔一定距离打印一次日志（避免刷屏）
        self.get_logger().info(
            f'[导航中] 剩余距离: {dist:.2f}m | '
            f'已用时: {nav_time}s | '
            f'恢复尝试: {recoveries}次',
            throttle_duration_sec=2.0  # 每2秒最多打印一次
        )

    # =========================================================================
    # Action 回调：导航完成
    # =========================================================================
    def _result_callback(self, future):
        """导航完成回调"""
        result = future.result()
        status = result.status

        with self._goal_lock:
            self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('导航成功！已到达目标点')
            self._set_status(NavStatus.REACHED)

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('导航已取消')
            self._set_status(NavStatus.CANCELLED)

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'导航失败（ABORTED）！状态码: {status}。'
                '可能原因：目标点不可达、路径规划失败、碰到障碍物'
            )
            self._set_status(NavStatus.FAILED)

        else:
            self.get_logger().warn(f'导航结束，未知状态: {status}')
            self._set_status(NavStatus.FAILED)

    # =========================================================================
    # Service 回调：取消导航
    # =========================================================================
    def _cancel_navigation_callback(self, request, response):
        """
        /cancel_navigation service 回调。
        取消当前正在执行的导航目标。
        """
        if self._current_status != NavStatus.NAVIGATING:
            response.success = False
            response.message = f'当前不在导航中，状态为: {self._current_status}'
            self.get_logger().warn(response.message)
            return response

        self._cancel_current_goal()
        response.success = True
        response.message = '导航取消请求已发送'
        self.get_logger().info(response.message)
        return response

    # =========================================================================
    # 内部方法：取消当前目标
    # =========================================================================
    def _cancel_current_goal(self):
        """取消当前活跃的 Nav2 goal"""
        with self._goal_lock:
            if self._goal_handle is not None:
                self.get_logger().info('正在取消 Nav2 导航目标...')
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        """取消完成回调"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Nav2 导航目标已成功取消')
        else:
            self.get_logger().warn('取消 Nav2 导航目标失败（可能已完成）')
        self._set_status(NavStatus.CANCELLED)

    # =========================================================================
    # 定时器：定期发布状态
    # =========================================================================
    def _publish_status(self):
        """定期发布导航状态到 /nav_status"""
        msg = String()
        msg.data = self._current_status
        self._status_pub.publish(msg)

    # =========================================================================
    # 内部方法：设置状态
    # =========================================================================
    def _set_status(self, status: str):
        """设置当前状态并立即发布"""
        old_status = self._current_status
        self._current_status = status
        if old_status != status:
            self.get_logger().info(f'导航状态变更: {old_status} -> {status}')
            # 立即发布状态变更
            msg = String()
            msg.data = status
            self._status_pub.publish(msg)

    # =========================================================================
    # 工具方法：四元数转偏航角
    # =========================================================================
    @staticmethod
    def _quat_to_yaw(q) -> float:
        """
        将四元数转换为偏航角(rad)。
        使用 atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


# =============================================================================
# Main
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    # 使用多线程执行器，支持 action client 和 subscription 并发
    executor = MultiThreadedExecutor(num_threads=4)

    node = NavigateToPointNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('navigate_to_point 节点已停止')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
