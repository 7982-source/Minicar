import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time

KEY_BINDINGS = {
    'u': (1, 1),    # 左上
    'i': (1, 0),    # 前进
    'o': (1, -1),   # 右上
    'j': (0, 1),    # 左转
    'l': (0, -1),   # 右转
    'm': (-1, 1),   # 左下
    ',': (-1, 0),   # 后退
    '.': (-1, -1),  # 右下
    'U': (1, 1, 1), # 平移左上
    'I': (1, 0, 1), # 平移前
    'O': (1, -1, 1),# 平移右上
    'J': (0, 1, 1), # 平移左
    'L': (0, -1, 1),# 平移右
    'M': (-1, 1, 1),# 平移左下
    '<': (-1, 0, 1),# 平移后
    '>': (-1, -1, 1),# 平移右下
    't': (0, 0, 1), # 上升
    'b': (0, 0, -1),# 下降
}

MSG = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It stops when no key is pressed.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


class TeleopTwistKeyboardStop(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard_stop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 速度参数
        self.max_linear = 1.0     # 最大线速度
        self.max_angular = 2.0    # 最大角速度
        self.linear = 0.0         # 当前线速度
        self.angular = 0.0        # 当前角速度
        self.linear_strafing = 0.0  # 平移速度

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        # 定时器（10ms周期）
        self.timer = self.create_timer(0.02, self.timer_callback)

        # 按键保持判断变量
        self.last_valid_key = ''  # 最后一次有效按键
        self.last_key_time = 0.0  # 最后一次有效按键时间（秒）
        self.key_hold_threshold = 0.05  # 按键保持阈值（50ms）

        self.get_logger().info(MSG)
        self.get_logger().info(f"当前速度限制：线速度 {self.max_linear:.2f} m/s，角速度 {self.max_angular:.2f} rad/s")

    def get_key(self):
        """非阻塞读取键盘输入"""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        return key

    def timer_callback(self):
        key = self.get_key()
        current_time = time.time()
        prev_linear = self.linear
        prev_angular = self.angular
        prev_strafing = self.linear_strafing

        # 更新最后一次有效按键
        if key in KEY_BINDINGS or key in ['q', 'z', 'w', 'x', 'e', 'c']:
            self.last_valid_key = key
            self.last_key_time = current_time
        elif key == '\x03':  # CTRL+C退出
            self.destroy_node()
            rclpy.shutdown()
            return

        # 判断是否为持续按键（未超时则沿用最后一次按键）
        time_since_last_key = current_time - self.last_key_time
        if time_since_last_key < self.key_hold_threshold and self.last_valid_key != '':
            key_to_use = self.last_valid_key
        else:
            key_to_use = ''
            self.last_valid_key = ''

        # 根据按键更新速度
        if key_to_use in KEY_BINDINGS:
            values = KEY_BINDINGS[key_to_use]
            if len(values) == 3:
                self.linear = values[0] * self.max_linear
                self.angular = values[1] * self.max_angular
                self.linear_strafing = values[2] * self.max_linear
            else:
                self.linear = values[0] * self.max_linear
                self.angular = values[1] * self.max_angular
                self.linear_strafing = 0.0
        elif key_to_use == 'q':
            self.max_linear *= 1.1
            self.max_angular *= 1.1
            self.get_logger().info(f"速度限制更新：线速度 {self.max_linear:.2f}，角速度 {self.max_angular:.2f}")
        elif key_to_use == 'z':
            self.max_linear *= 0.9
            self.max_angular *= 0.9
            self.get_logger().info(f"速度限制更新：线速度 {self.max_linear:.2f}，角速度 {self.max_angular:.2f}")
        elif key_to_use == 'w':
            self.max_linear *= 1.1
            self.get_logger().info(f"线速度限制更新：{self.max_linear:.2f}")
        elif key_to_use == 'x':
            self.max_linear *= 0.9
            self.get_logger().info(f"线速度限制更新：{self.max_linear:.2f}")
        elif key_to_use == 'e':
            self.max_angular *= 1.1
            self.get_logger().info(f"角速度限制更新：{self.max_angular:.2f}")
        elif key_to_use == 'c':
            self.max_angular *= 0.9
            self.get_logger().info(f"角速度限制更新：{self.max_angular:.2f}")
        else:
            self.linear = 0.0
            self.angular = 0.0
            self.linear_strafing = 0.0

        # 速度变化时发布指令
        if (self.linear != prev_linear or 
            self.angular != prev_angular or 
            self.linear_strafing != prev_strafing):
        # key = self.get_key()
        # if key in KEY_BINDINGS or key in ['q', 'z', 'w', 'x', 'e', 'c']:
        #     self.last_valid_key = key
            
        # elif key == '\x03':  # CTRL+C退出
        #     self.destroy_node()
        #     rclpy.shutdown()
        #     return

            twist = Twist()
            twist.linear.x = self.linear
            # twist.linear.x += 1
            twist.linear.y = self.linear_strafing
            twist.linear.z += 1
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.angular
            self.publisher_.publish(twist)
            # 打印调试信息（查看是否持续输出300 0）
            self.get_logger().debug(f"发布速度：{self.linear} {self.angular}")

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopTwistKeyboardStop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()