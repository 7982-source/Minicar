import serial
import struct
import rclpy
import math
import sys
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# ==============================================================================
# 【全局配置区】请根据硬件实际参数修改！
# ==============================================================================
# -------------------------- 1. 串口基础配置 --------------------------
SERIAL_PORT = '/dev/ttyUSB0'  # Linux: /dev/ttyUSBx; Windows: COMx
BAUD_RATE = 115200            # 必须与STM32端波特率一致
SERIAL_TIMEOUT = 0.1          # 串口读写超时时间（秒）

# -------------------------- 2. 指令下发协议（Twist -> 串口） --------------------------
# 下发数据包结构：包头(0xFF) + 线性速度(int16) + 角速度(int16) + 包尾(0xFE) → 共6字节
SEND_HEADER = 0xFF
SEND_FOOTER = 0xFE
# 速度映射范围（根据实际需求调整：输入Twist范围 → 输出int16范围）
LINEAR_TWIST_MIN = -1.0       # 输入线性速度最小值（m/s）
LINEAR_TWIST_MAX = 1.0        # 输入线性速度最大值（m/s）
ANGULAR_TWIST_MIN = -2.0      # 输入角速度最小值（rad/s）
ANGULAR_TWIST_MAX = 2.0       # 输入角速度最大值（rad/s）
SEND_DATA_MIN = -500           # 输出int16最小值（用户原配置，注意：若min=max会导致数据无变化）
SEND_DATA_MAX = 500           # 输出int16最大值（建议实际使用时改为-32768~32767）

# -------------------------- 3. 数据上传协议（串口 -> Odometry） --------------------------
# 接收数据包结构：包头(0x55) + 左轮编码器(int8) + 右轮编码器(int8) + 包尾(0x0D) → 共4字节
RECV_HEADER = 0x55
RECV_FOOTER = 0x0D
RECV_PACKET_SIZE = 4          # 接收数据包固定长度
RECV_PACKET_FORMAT = '<BbbB'  # 解包格式：1字节头 + 2字节编码器 + 1字节尾（小端）

# -------------------------- 4. 里程计物理参数 --------------------------
WHEEL_BASE = 0.17             # 左右轮距（米）
WHEEL_CIRCUMFERENCE = 0.215   # 轮子周长（米）
TICKS_PER_REVOLUTION = 2640   # 编码器每转脉冲数
BOARD_SEND_INTERVAL = 0.0050  # STM32端发送数据的时间间隔（秒）
TICKS_PER_METER = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE  # 每米对应脉冲数

# -------------------------- 5. ROS2 坐标系配置 --------------------------
ODOM_FRAME = 'odom'           # 里程计父坐标系
BASE_FRAME = 'base_footprint'      # 机器人基坐标系


# ==============================================================================
# 【工具函数】数据映射、四元数转换
# ==============================================================================
def map_value(x, in_min, in_max, out_min, out_max) -> int:
    """将输入值从一个范围线性映射到另一个范围，并限制边界"""
    x_clamped = max(in_min, min(x, in_max))  # 防止输入超出范围
    return int((x_clamped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def euler_to_quaternion(yaw: float) -> tuple:
    """将偏航角（yaw，绕z轴旋转）转换为四元数（x,y,z,w）"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)  # roll=0, pitch=0，仅保留yaw


# ==============================================================================
# 【核心节点】融合指令下发与Odometry发布
# ==============================================================================
class SerialCmdOdomNode(Node):
    def __init__(self):
        super().__init__('serial_cmd_odom_node')
        self.get_logger().info("=== 串口指令-Odometry融合节点启动 ===")
        self.cmd_subs = 0  # 替代全局变量n，用于计数下发的指令
        self.cmd_pub = 0
        # -------------------------- 1. 初始化串口（共享实例） --------------------------
        self.ser = None
        self.buffer = b''  # 接收数据缓冲区
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=SERIAL_TIMEOUT,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,  # 必须改为 STOPBITS_ONE
                bytesize=serial.EIGHTBITS
            )
            self.get_logger().info(f"✅ 串口初始化成功：{SERIAL_PORT} @ {BAUD_RATE} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 串口初始化失败：{e}")
            self.get_logger().error("请检查：1.串口端口 2.波特率 3.串口是否被占用")
            sys.exit(1)

        # -------------------------- 2. ROS2 发布器/订阅器/定时器 --------------------------
        # 2.1 订阅cmd_vel（接收Twist指令）
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # 消息队列长度
        )

        # 2.2 发布Odometry和TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2.3 定时器（5ms循环读取串口，解析编码器数据）
        self.read_timer = self.create_timer(0.02, self.serial_read_parse)

        # -------------------------- 3. 里程计状态变量 --------------------------
        self.x = 0.0          # 机器人x坐标（全局）
        self.y = 0.0          # 机器人y坐标（全局）
        self.yaw = 0.0        # 机器人偏航角（全局，范围[-π,π]）
        self.last_odom_time = self.get_clock().now()  # 上一次里程计更新时间

    

    def cmd_vel_callback(self, msg: Twist):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("⚠️ 串口未打开，无法下发指令")
            return

        # 1. 逆运动学解算：机器人速度 → 左右轮线速度（m/s）（保留原逻辑，确保正确）
        v = msg.linear.x          # 机器人前进速度（m/s）
        omega = msg.angular.z     # 机器人转向角速度（rad/s）
        L = WHEEL_BASE            # 左右轮距（0.17m）
        v_left = v - (omega * L) / 2   # 左轮线速度（m/s）
        v_right = v + (omega * L) / 2  # 右轮线速度（m/s）

        # 2. 【核心修改：用物理公式计算单次脉冲数，替换原map_value】
        # 提取全局配置中的硬件参数（确保与实际一致）
        wheel_circum = WHEEL_CIRCUMFERENCE    # 车轮周长（0.215m）
        ticks_per_rev = TICKS_PER_REVOLUTION   # 编码器每转脉冲数（2640）
        send_interval = BOARD_SEND_INTERVAL    # 指令下发间隔（0.005s）

        # 计算左轮单次脉冲数（公式：single_ticks = (v_wheel * ticks_per_rev * send_interval) / wheel_circum）

        #注意，我們發送的脈衝是小車端的2.5     
        if abs(v_left) < 0.001:  # 速度接近0时，脉冲数归0（避免微小脉冲导致电机抖动）
            left_ticks = 0
        else:
            left_ticks = int(( (v_left * ticks_per_rev * send_interval) / wheel_circum )*1)

        # 计算右轮单次脉冲数（同上）
        if abs(v_right) < 0.001:
            right_ticks = 0
        else:
            right_ticks = int(( (v_right * ticks_per_rev * send_interval) / wheel_circum )*1)

        # 3. 脉冲数限制：避免超出下位机支持的最大脉冲范围（根据你的STM32配置调整，如±100）
        MAX_TICKS = 100  # 假设下位机单次最大接收100脉冲（需根据实际修改！）
        left_ticks = max(-MAX_TICKS, min(left_ticks, MAX_TICKS))  # 限制在±MAX_TICKS
        right_ticks = max(-MAX_TICKS, min(right_ticks, MAX_TICKS))

        # 4. 打印调试：查看脉冲数是否符合预期
        self.cmd_pub += 1
        print(f"pub {time.strftime('%H:%M:%S')}] {self.cmd_pub} 左轮脉冲={left_ticks}, 右轮脉冲={right_ticks}")

        # 5. 构造脉冲指令数据包（包头+左轮脉冲+右轮脉冲+包尾，与STM32协议一致）
        try:
            # 打包“左轮脉冲数”和“右轮脉冲数”（int16大端模式，顺序需与STM32一致）
            payload = struct.pack('>hh', left_ticks, right_ticks)  # 关键：用脉冲数替代原映射值
            send_packet = bytes([SEND_HEADER]) + payload + bytes([SEND_FOOTER])
        except struct.error as e:
            self.get_logger().error(f"❌ 脉冲指令打包失败：{e}")
            return

        # 6. 发送脉冲指令到串口
        try:
            self.ser.write(send_packet)
            self.get_logger().debug(
                f"📤 下发脉冲：左轮={v_left:.2f}m/s→{left_ticks}ticks, 右轮={v_right:.2f}m/s→{right_ticks}ticks | Hex: {send_packet.hex().upper()}"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 脉冲指令 #{self.cmd_pub} 发送失败：{e}")


    # -------------------------- 【功能2：读取串口，解析Odometry】 --------------------------
    def serial_read_parse(self):
        """读取串口数据，解析编码器数据包，计算并发布Odometry和TF"""
        if not self.ser or not self.ser.is_open:
            return

        # 1. 读取串口所有可用数据，存入缓冲区
        try:
            new_data = self.ser.read(self.ser.in_waiting or 1)
            if new_data:
                self.buffer += new_data
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 串口读取错误：{e}")
            return

        # 2. 解析缓冲区中的数据包（找包头、验包尾、提数据）
        while True:
            # 2.1 查找包头（0x55），同步数据包
            header_idx = self.buffer.find(bytes([RECV_HEADER]))
            if header_idx == -1:
                # 无包头：缓冲区过大时清空，避免内存占用
                if len(self.buffer) > RECV_PACKET_SIZE * 2:
                    self.buffer = b''
                break

            # 2.2 从包头开始截取数据，检查是否足够一个包长度
            self.buffer = self.buffer[header_idx:]
            if len(self.buffer) < RECV_PACKET_SIZE:
                break  # 数据不足，等待下一次读取

            # 2.3 验证包尾（最后一字节是否为0x0D）
            if self.buffer[RECV_PACKET_SIZE - 1] != RECV_FOOTER:
                self.buffer = self.buffer[1:]  # 包尾错误，丢弃当前包头，继续找下一个
                continue

            # 2.4 解析有效数据包
            valid_packet = self.buffer[:RECV_PACKET_SIZE]
            self.buffer = self.buffer[RECV_PACKET_SIZE:]  # 移除已解析数据
            self.calculate_odometry(valid_packet)  # 计算并发布里程计


    def calculate_odometry(self, packet: bytes):
        """解析编码器数据包，计算里程计并发布"""
        # 1. 解包编码器数据（包头0x55 + 左轮int8 + 右轮int8 + 包尾0x0D）
        try:
            recv_data = struct.unpack(RECV_PACKET_FORMAT, packet)
            left_ticks = recv_data[1]   # 左轮编码器脉冲数（int8）
            right_ticks = recv_data[2]  # 右轮编码器脉冲数（int8）
        except struct.error as e:
            self.get_logger().warn(f"❌ 编码器数据解包失败：{e}")
            return

        # 2. 计算左右轮线速度（基于编码器脉冲和STM32发送间隔）
        # 2.1 脉冲→距离：脉冲数 / 每米脉冲数
        left_distance = left_ticks / TICKS_PER_METER
        right_distance = right_ticks / TICKS_PER_METER
        # 2.2 距离→速度：距离 / STM32发送间隔
        left_speed = left_distance / BOARD_SEND_INTERVAL
        right_speed = right_distance / BOARD_SEND_INTERVAL

        # 3. 差速底盘运动学：计算机器人线性速度和角速度
        linear_x = (left_speed + right_speed) / 2.0    # 机器人前进速度
        angular_z = (right_speed - left_speed) / WHEEL_BASE  # 机器人转向角速度

        # 4. 积分计算机器人全局位姿（时间差基于ROS时钟）
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_odom_time).nanoseconds / 1e9  # 时间差（秒）
        self.last_odom_time = current_time

        if time_diff > 0:
            # 4.1 计算位姿变化量（基于当前速度和时间差）
            delta_x = linear_x * math.cos(self.yaw) * time_diff  # x方向变化
            delta_y = linear_x * math.sin(self.yaw) * time_diff  # y方向变化
            delta_yaw = angular_z * time_diff                   # 偏航角变化

            # 4.2 更新全局位姿（积分）
            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw
            # 限制偏航角在[-π, π]范围
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.cmd_subs +=1 
        print(f"subodom:[{time.strftime('%H:%M:%S')}] {self.cmd_subs}: 左轮脉冲={left_ticks}, 右轮脉冲={right_ticks}, 左线速度={left_speed:.2f}m/s, 右线速度={right_speed:.2f}m/s")
        # 5. 发布Odometry消息
        self.publish_odometry(current_time, linear_x, angular_z)
        # 6. 发布odom→base_link的TF变换
        self.publish_tf(current_time)


    def publish_odometry(self, current_time, linear_x: float, angular_z: float):
        """发布Odometry消息"""
        odom_msg = Odometry()
        # 消息头
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = ODOM_FRAME
        odom_msg.child_frame_id = BASE_FRAME

        # 位置信息（x,y,z + 四元数）
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = euler_to_quaternion(self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # 速度信息（线性x + 角速度z）
        odom_msg.twist.twist.linear.x = linear_x
        odom_msg.twist.twist.angular.z = angular_z

        # 协方差（根据实际精度调整，用于EKF融合）
        odom_msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        odom_msg.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.odom_pub.publish(odom_msg)


    def publish_tf(self, current_time):
        """发布odom→base_link的TF变换（用于RViz可视化和坐标转换）"""
        tf_msg = TransformStamped()
        # TF头信息
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = ODOM_FRAME
        tf_msg.child_frame_id = BASE_FRAME

        # 位置变换
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        # 姿态变换（四元数）
        quat = euler_to_quaternion(self.yaw)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)


    # -------------------------- 【节点销毁】释放资源 --------------------------
    def destroy_node(self):
        """关闭串口，释放ROS2资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("🔌 串口已关闭")
        super().destroy_node()


# ==============================================================================
# 【节点启动入口】
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdOdomNode()
    try:
        rclpy.spin(node)  # 阻塞运行节点
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()