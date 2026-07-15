#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

class MotionDemoNode(Node):
    def __init__(self):
        super().__init__('motion_demo_node')
        
        # Khởi tạo publisher đưa lệnh điều khiển tới /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Khai báo các tham số điều hướng di chuyển
        self.declare_parameter('mode', 'stop') # Chế độ di chuyển mặc định
        self.declare_parameter('linear_speed', 0.25) # Vận tốc dài m/s
        self.declare_parameter('angular_speed', 0.4) # Vận tốc góc rad/s
        
        # Đọc giá trị cấu hình ban đầu
        self.mode = self.get_parameter('mode').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        self.get_logger().info(f"Node Motion Demo đã được khởi động ở chế độ: [{self.mode.upper()}]")
        self.get_logger().info(f"Các tham số: Speed={self.linear_speed}m/s, TurnSpeed={self.angular_speed}rad/s")
        
        # Chu kỳ chạy tính toán logic điều khiển là 10Hz (0.1 giây)
        self.control_period = 0.1
        self.create_timer(self.control_period, self.control_loop)

    def control_loop(self):
        # Đọc động parameter phòng trường hợp người dùng thay đổi ở runtime
        self.mode = self.get_parameter('mode').value.lower()
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        twist = Twist()
        
        # Các chế độ di chuyển cơ bản liên tục
        if self.mode == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.mode == 'forward':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
        elif self.mode == 'backward':
            twist.linear.x = -self.linear_speed
            twist.angular.z = 0.0
            
        elif self.mode == 'left':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            
        elif self.mode == 'right':
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed

        else:
            self.get_logger().warn(f"Chế độ [{self.mode}] không được hỗ trợ!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Gửi lệnh
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Gửi lệnh dừng xe nhiều lần liên tiếp để đảm bảo DDS truyền tải thành công."""
        import time
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Đang gửi lệnh dừng xe khẩn cấp...")
        for i in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.get_logger().info("Đã dừng xe an toàn.")

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionDemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.stop_robot()
    finally:
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
