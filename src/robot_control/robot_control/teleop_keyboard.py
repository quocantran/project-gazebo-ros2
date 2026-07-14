#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time

# Hướng dẫn sử dụng trên Terminal
USAGE = """
--------------------------------------------------
HỆ THỐNG ĐIỀU KHIỂN BÀN PHÍM CHUẨN ROBOT CAR
--------------------------------------------------
Giữ phím để di chuyển - Nhả phím ra xe tự động DỪNG!

Các phím đi thẳng / rẽ:
        q (Tiến-Trái)   w (Tiến)    e (Tiến-Phải)
        a (Xoay-Trái)   s (Lùi)     d (Xoay-Phải)
        z (Lùi-Trái)                c (Lùi-Phải)

Phím dừng khẩn cấp: x

Thay đổi vận tốc (Mặc định):
- Tốc độ dài (Linear): 0.3 m/s
- Tốc độ góc (Angular): 0.5 rad/s

Nhấn Ctrl+C để thoát an toàn.
--------------------------------------------------
"""

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # Khởi tạo publisher đưa lệnh điều khiển động cơ tới /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Khai báo các parameters điều chỉnh tốc độ
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Lưu trữ cấu hình Terminal gốc để khôi phục khi tắt node (chỉ khi chạy trực tiếp trong TTY)
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None
            self.get_logger().error(
                "\n========================================================================\n"
                "LỖI: Node này cần nhận dữ liệu bàn phím trực tiếp từ Terminal.\n"
                "Khi chạy qua 'ros2 launch', Standard Input bị chặn lại.\n"
                "Vui lòng chạy trực tiếp bằng lệnh sau ở Terminal mới:\n"
                "  ros2 run robot_control teleop_keyboard\n"
                "========================================================================="
            )
            raise SystemExit("Chạy node teleop_keyboard qua 'ros2 run' thay vì 'ros2 launch'")
        
        # Biến quản lý trạng thái watchdog phím bấm
        self.last_key = ''
        self.last_key_time = time.time()
        self.key_timeout = 0.25  # Giây (Nếu quá thời gian này không nhận được phím lặp lại -> nhả phím -> dừng)
        
        # Trạng thái tốc độ hiện tại để tránh gửi trùng lặp liên tục
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.get_logger().info("Khởi động Node Teleop Keyboard thành công.")
        print(USAGE)
        
        # Tạo timer tần số cao (20Hz) để liên tục kiểm tra phím bấm và watchdog
        self.create_timer(0.05, self.keyboard_loop)

    def get_key(self):
        """Đọc một ký tự từ bàn phím dạng không chặn (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        # Chờ tối đa 0.02 giây để đọc ký tự, phản hồi cực nhạy
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        key = self.get_key()
        now = time.time()
        
        if key:
            if key == '\x03':  # Ctrl+C
                self.get_logger().info("Nhận tín hiệu tắt node từ bàn phím.")
                raise KeyboardInterrupt
            
            # Nếu là phím di chuyển hợp lệ, cập nhật trạng thái
            if key in ['w', 's', 'a', 'd', 'q', 'e', 'z', 'c', 'x']:
                self.last_key = key
                self.last_key_time = now
        else:
            # Nếu không bấm phím nào, kiểm tra xem đã quá thời gian timeout (nhả phím) chưa
            if now - self.last_key_time > self.key_timeout:
                self.last_key = ''
        
        # Tính toán vận tốc gửi cho robot dựa trên phím bấm hiện tại
        target_linear_x = 0.0
        target_angular_z = 0.0
        
        if self.last_key == 'w':
            target_linear_x = self.linear_speed
            target_angular_z = 0.0
        elif self.last_key == 's':
            target_linear_x = -self.linear_speed
            target_angular_z = 0.0
        elif self.last_key == 'a':
            target_linear_x = 0.0
            target_angular_z = self.angular_speed
        elif self.last_key == 'd':
            target_linear_x = 0.0
            target_angular_z = -self.angular_speed
        elif self.last_key == 'q':
            target_linear_x = self.linear_speed
            target_angular_z = self.angular_speed
        elif self.last_key == 'e':
            target_linear_x = self.linear_speed
            target_angular_z = -self.angular_speed
        elif self.last_key == 'z':
            target_linear_x = -self.linear_speed
            target_angular_z = self.angular_speed
        elif self.last_key == 'c':
            target_linear_x = -self.linear_speed
            target_angular_z = -self.angular_speed
        elif self.last_key == 'x':
            target_linear_x = 0.0
            target_angular_z = 0.0
            
        # Gửi lệnh lên cmd_vel nếu có sự thay đổi vận tốc
        if target_linear_x != self.current_linear_x or target_angular_z != self.current_angular_z:
            self.current_linear_x = target_linear_x
            self.current_angular_z = target_angular_z
            
            twist = Twist()
            twist.linear.x = self.current_linear_x
            twist.angular.z = self.current_angular_z
            self.cmd_vel_pub.publish(twist)
            
            # Log trạng thái di chuyển
            if self.last_key == '':
                self.get_logger().info("Nhả phím -> DỪNG XE")
            else:
                self.get_logger().info(
                    f"Di chuyển: Linear={self.current_linear_x:.2f} m/s, Angular={self.current_angular_z:.2f} rad/s"
                )

    def stop_robot(self):
        """Gửi lệnh dừng xe trước khi tắt hẳn node"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Đã gửi lệnh dừng xe dừng khẩn cấp.")
        # Thêm sleep ngắn để DDS kịp đẩy gói tin dừng xe đi trước khi Shutdown
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.stop_robot()
    finally:
        # Khôi phục trạng thái terminal chuẩn ban đầu
        try:
            if node.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
