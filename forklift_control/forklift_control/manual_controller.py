#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
叉车手动控制节点（键盘控制）
==============================
使用 Python curses 库实现键盘控制，适用于仿真测试场景。

控制键位说明：
  移动控制（WASD）：
    W / ↑  : 前进
    S / ↓  : 后退
    A / ←  : 左转
    D / →  : 右转
    空格键  : 紧急停止

  货叉控制（数字键）：
    1      : 货叉降至最低位（0.0）
    2      : 货叉降至取货位（0.1）
    3      : 货叉升至搬运位（0.2）
    4      : 货叉升至中位（0.5）
    5      : 货叉升至最高位（1.0）
    +/=    : 货叉上升 0.1
    -      : 货叉下降 0.1

  系统控制：
    Q / q  : 退出程序

发布：
  /cmd_vel        (geometry_msgs/Twist)  — 运动控制指令
  /fork_position  (std_msgs/Float64)     — 货叉高度指令

ROS2 Humble 环境
"""

import curses
import sys
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


# ============================================================
# 控制参数
# ============================================================

# 线速度档位（米/秒）
LINEAR_SPEED_LEVELS = [0.3, 0.6, 1.0, 1.5, 2.0]
# 角速度档位（弧度/秒）
ANGULAR_SPEED_LEVELS = [0.3, 0.5, 0.8, 1.2, 1.5]
# 默认速度档位（索引）
DEFAULT_SPEED_LEVEL = 1

# 货叉高度预设值（归一化 0~1）
FORK_PRESETS = {
    '1': (0.0,  '最低位（0%）'),
    '2': (0.1,  '取货位（10%）'),
    '3': (0.2,  '搬运位（20%）'),
    '4': (0.5,  '中位（50%）'),
    '5': (1.0,  '最高位（100%）'),
}

# 货叉步进量
FORK_STEP = 0.1


# ============================================================
# 手动控制节点
# ============================================================
class ManualController(Node):
    """叉车手动控制节点"""

    def __init__(self):
        super().__init__('manual_controller')

        # ---------- 内部状态 ----------
        self._linear_vel = 0.0       # 当前线速度指令
        self._angular_vel = 0.0      # 当前角速度指令
        self._fork_position = 0.0    # 当前货叉高度（归一化）
        self._speed_level = DEFAULT_SPEED_LEVEL  # 速度档位
        self._current_odom_x = 0.0
        self._current_odom_y = 0.0
        self._current_odom_yaw = 0.0
        self._fork_status = 'UNKNOWN'

        # 按键状态（用于连续按键控制）
        self._key_forward = False
        self._key_backward = False
        self._key_left = False
        self._key_right = False

        # 运行标志
        self._running = True

        # ---------- 发布者 ----------
        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_fork_pos = self.create_publisher(Float64, '/fork_position', 10)

        # ---------- 订阅者（用于状态显示）----------
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10
        )
        self._sub_fork_status = self.create_subscription(
            String, '/fork_status', self._cb_fork_status, 10
        )

        # ---------- 控制发布定时器（20Hz）----------
        self._pub_timer = self.create_timer(0.05, self._publish_cmd)

        self.get_logger().info('手动控制节点已启动，请在终端按键控制叉车')

    # ============================================================
    # 回调函数
    # ============================================================

    def _cb_odom(self, msg: Odometry):
        """接收里程计数据"""
        self._current_odom_x = msg.pose.pose.position.x
        self._current_odom_y = msg.pose.pose.position.y

        # 提取偏航角
        q = msg.pose.pose.orientation
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_odom_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _cb_fork_status(self, msg: String):
        """接收货叉状态"""
        self._fork_status = msg.data

    # ============================================================
    # 控制发布
    # ============================================================

    def _publish_cmd(self):
        """定期发布控制指令（20Hz）"""
        # 根据按键状态计算速度
        linear = 0.0
        angular = 0.0

        lin_speed = LINEAR_SPEED_LEVELS[self._speed_level]
        ang_speed = ANGULAR_SPEED_LEVELS[self._speed_level]

        if self._key_forward:
            linear = lin_speed
        elif self._key_backward:
            linear = -lin_speed

        if self._key_left:
            angular = ang_speed
        elif self._key_right:
            angular = -ang_speed

        self._linear_vel = linear
        self._angular_vel = angular

        # 发布速度指令
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._pub_cmd_vel.publish(twist)

    def _stop(self):
        """紧急停止"""
        self._key_forward = False
        self._key_backward = False
        self._key_left = False
        self._key_right = False
        twist = Twist()
        self._pub_cmd_vel.publish(twist)

    def _set_fork_position(self, position: float):
        """设置货叉高度并发布"""
        self._fork_position = max(0.0, min(1.0, position))
        msg = Float64()
        msg.data = self._fork_position
        self._pub_fork_pos.publish(msg)

    # ============================================================
    # curses 界面
    # ============================================================

    def run_keyboard_control(self, stdscr):
        """
        curses 主循环
        处理键盘输入并更新显示界面
        """
        # 初始化 curses
        curses.curs_set(0)          # 隐藏光标
        stdscr.nodelay(True)        # 非阻塞读取
        stdscr.timeout(50)          # 50ms 刷新
        curses.start_color()
        curses.use_default_colors()

        # 定义颜色对
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # 正常状态
        curses.init_pair(2, curses.COLOR_YELLOW, -1)  # 警告
        curses.init_pair(3, curses.COLOR_RED, -1)     # 错误/危险
        curses.init_pair(4, curses.COLOR_CYAN, -1)    # 信息
        curses.init_pair(5, curses.COLOR_WHITE, -1)   # 普通文字

        COLOR_GREEN  = curses.color_pair(1)
        COLOR_YELLOW = curses.color_pair(2)
        COLOR_RED    = curses.color_pair(3)
        COLOR_CYAN   = curses.color_pair(4)
        COLOR_WHITE  = curses.color_pair(5)

        last_key_name = ''
        frame_count = 0

        while self._running:
            frame_count += 1

            # ---- 读取按键 ----
            try:
                key = stdscr.getch()
            except Exception:
                key = -1

            # 处理普通字符键
            key_pressed = False
            if key != -1:
                last_key_name = self._process_key(key)
                key_pressed = True

            # ---- 绘制界面 ----
            try:
                stdscr.erase()
                max_y, max_x = stdscr.getmaxyx()

                row = 0

                # 标题栏
                title = ' 叉车手动控制终端 '
                title_x = max(0, (max_x - len(title)) // 2)
                stdscr.addstr(row, title_x, title,
                              COLOR_CYAN | curses.A_BOLD | curses.A_REVERSE)
                row += 2

                # ---- 状态显示区 ----
                stdscr.addstr(row, 2, '─' * (max_x - 4), COLOR_CYAN)
                row += 1
                stdscr.addstr(row, 2, ' 实时状态 ', COLOR_CYAN | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, 2, '─' * (max_x - 4), COLOR_CYAN)
                row += 1

                # 速度信息
                speed_color = COLOR_GREEN if (
                    abs(self._linear_vel) > 0.01 or
                    abs(self._angular_vel) > 0.01
                ) else COLOR_WHITE

                stdscr.addstr(row, 4, f'线速度: {self._linear_vel:+.2f} m/s', speed_color)
                stdscr.addstr(row, 30, f'角速度: {self._angular_vel:+.2f} rad/s', speed_color)
                row += 1

                # 位置信息
                stdscr.addstr(row, 4,
                    f'位置: X={self._current_odom_x:.2f}m  '
                    f'Y={self._current_odom_y:.2f}m  '
                    f'偏航={self._current_odom_yaw:.1f}°',
                    COLOR_WHITE)
                row += 1

                # 货叉信息
                fork_bar_len = 20
                fork_filled = int(self._fork_position * fork_bar_len)
                fork_bar = ('█' * fork_filled + '░' * (fork_bar_len - fork_filled))
                fork_color = (COLOR_YELLOW
                              if self._fork_status == 'MOVING'
                              else COLOR_GREEN)
                stdscr.addstr(row, 4,
                    f'货叉高度: [{fork_bar}] {self._fork_position*100:.0f}%'
                    f'  [{self._fork_status}]',
                    fork_color)
                row += 1

                # 速度档位
                speed_bar = '◆' * (self._speed_level + 1) + '◇' * (4 - self._speed_level)
                lin_speed = LINEAR_SPEED_LEVELS[self._speed_level]
                ang_speed = ANGULAR_SPEED_LEVELS[self._speed_level]
                stdscr.addstr(row, 4,
                    f'速度档位: [{speed_bar}] 线速={lin_speed}m/s  角速={ang_speed}rad/s',
                    COLOR_WHITE)
                row += 1

                # 最后按键
                stdscr.addstr(row, 4, f'最后按键: {last_key_name}', COLOR_YELLOW)
                row += 2

                # ---- 操作说明 ----
                stdscr.addstr(row, 2, '─' * (max_x - 4), COLOR_CYAN)
                row += 1
                stdscr.addstr(row, 2, ' 操作说明 ', COLOR_CYAN | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, 2, '─' * (max_x - 4), COLOR_CYAN)
                row += 1

                # 移动控制
                stdscr.addstr(row, 4, '移动控制:', COLOR_WHITE | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, 6, 'W/↑  前进    S/↓  后退    A/←  左转    D/→  右转',
                              COLOR_WHITE)
                row += 1
                stdscr.addstr(row, 6, '空格  紧急停止    PageUp  提速    PageDown  降速',
                              COLOR_WHITE)
                row += 1

                # 货叉控制
                stdscr.addstr(row, 4, '货叉控制:', COLOR_WHITE | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, 6, '1:最低  2:取货位  3:搬运位  4:中位  5:最高',
                              COLOR_WHITE)
                row += 1
                stdscr.addstr(row, 6, '+/=  货叉上升 10%    -  货叉下降 10%',
                              COLOR_WHITE)
                row += 1

                # 系统
                stdscr.addstr(row, 4, '系统:', COLOR_WHITE | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, 6, 'Q  退出程序', COLOR_RED)
                row += 2

                # 底部状态栏
                bottom_msg = f' FPS:{frame_count%100:03d} | 按 Q 退出 '
                if row < max_y - 1:
                    stdscr.addstr(max_y - 1, 0,
                                  bottom_msg[:max_x - 1].ljust(max_x - 1),
                                  COLOR_CYAN | curses.A_REVERSE)

                stdscr.refresh()

            except curses.error:
                # 终端窗口过小时忽略绘制错误
                pass

        # 退出时停止叉车
        self._stop()

    def _process_key(self, key: int) -> str:
        """
        处理键盘输入，更新控制状态
        返回按键名称字符串（用于显示）
        """
        # ---------- W/S/A/D 移动控制 ----------
        if key in (ord('w'), ord('W'), curses.KEY_UP):
            self._key_forward = True
            self._key_backward = False
            return 'W/↑ 前进'

        elif key in (ord('s'), ord('S'), curses.KEY_DOWN):
            self._key_backward = True
            self._key_forward = False
            return 'S/↓ 后退'

        elif key in (ord('a'), ord('A'), curses.KEY_LEFT):
            self._key_left = True
            self._key_right = False
            return 'A/← 左转'

        elif key in (ord('d'), ord('D'), curses.KEY_RIGHT):
            self._key_right = True
            self._key_left = False
            return 'D/→ 右转'

        # ---------- 按键释放（松开方向键时停止）----------
        # curses 非阻塞模式下，实现方式：
        # 如果连续两帧没有按任何移动键，则清除移动标志
        # 这里用一个简化处理：空格键停止
        elif key == ord(' '):
            self._stop()
            return '空格 急停'

        # ---------- 速度档位 ----------
        elif key == curses.KEY_PPAGE:  # Page Up
            self._speed_level = min(4, self._speed_level + 1)
            return f'PageUp 提速→档{self._speed_level+1}'

        elif key == curses.KEY_NPAGE:  # Page Down
            self._speed_level = max(0, self._speed_level - 1)
            return f'PageDn 降速→档{self._speed_level+1}'

        # ---------- 货叉高度预设 ----------
        elif key == ord('1'):
            self._set_fork_position(0.0)
            return '1 货叉最低位'

        elif key == ord('2'):
            self._set_fork_position(0.1)
            return '2 货叉取货位'

        elif key == ord('3'):
            self._set_fork_position(0.2)
            return '3 货叉搬运位'

        elif key == ord('4'):
            self._set_fork_position(0.5)
            return '4 货叉中位'

        elif key == ord('5'):
            self._set_fork_position(1.0)
            return '5 货叉最高位'

        # ---------- 货叉步进 ----------
        elif key in (ord('+'), ord('=')):
            self._set_fork_position(self._fork_position + FORK_STEP)
            return f'+ 货叉上升→{self._fork_position*100:.0f}%'

        elif key == ord('-'):
            self._set_fork_position(self._fork_position - FORK_STEP)
            return f'- 货叉下降→{self._fork_position*100:.0f}%'

        # ---------- 退出 ----------
        elif key in (ord('q'), ord('Q')):
            self._running = False
            return 'Q 退出'

        return f'key={key}'


# ============================================================
# ROS2 spin 线程
# ============================================================

def ros_spin_thread(node: ManualController):
    """在独立线程中运行 ROS2 spin"""
    try:
        rclpy.spin(node)
    except Exception:
        pass


# ============================================================
# 主入口
# ============================================================

def main(args=None):
    """手动控制节点主入口"""
    rclpy.init(args=args)

    node = ManualController()

    # 在独立线程中运行 ROS2 spin，主线程运行 curses 界面
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # 使用 curses.wrapper 安全启动 curses 界面
        # wrapper 会在发生异常时自动恢复终端状态
        curses.wrapper(node.run_keyboard_control)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'\n控制界面异常: {e}')
    finally:
        # 确保停止叉车
        node._stop()
        node.get_logger().info('手动控制节点关闭')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print('\n叉车已停止，程序退出。')


if __name__ == '__main__':
    main()
