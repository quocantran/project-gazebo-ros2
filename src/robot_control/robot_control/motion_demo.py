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
        self.declare_parameter('side_duration', 4.0) # Thời gian đi thẳng 1 cạnh hình vuông (giây)
        self.declare_parameter('turn_duration', 3.14) # Thời gian xoay 90 độ (tùy chỉnh phụ thuộc tốc độ góc, ví dụ 1.57 / angular_speed = 3.9s)
        self.declare_parameter('circle_duration', 10.0) # Thời gian chạy hết 1 vòng (giây)
        
        # Đọc giá trị cấu hình ban đầu
        self.mode = self.get_parameter('mode').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.side_duration = self.get_parameter('side_duration').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.circle_duration = self.get_parameter('circle_duration').value
        
        # Các biến phục vụ máy trạng thái (State Machine)
        self.state_time = 0.0
        self.state = "INIT" # Trạng thái trong kịch bản di chuyển phức tạp
        self.sub_state = "FORWARD" # Phục vụ cho hình vuông
        self.eight_direction = 1.0 # 1.0: Trái, -1.0: Phải cho hình số 8
        
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
        self.side_duration = self.get_parameter('side_duration').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.circle_duration = self.get_parameter('circle_duration').value
        
        twist = Twist()
        
        # 1. Các chế độ di chuyển cơ bản liên tục
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

        # 2. Chế độ chạy đường tròn (Circle)
        elif self.mode == 'circle':
            twist.linear.x = self.linear_speed
            # R = v / w -> w = v / R. Ở đây ta chọn R = 0.75m -> w = v / 0.75
            twist.angular.z = self.linear_speed / 0.75

        # 3. Chế độ chạy hình vuông (Square)
        elif self.mode == 'square':
            self.state_time += self.control_period
            
            if self.sub_state == "FORWARD":
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                if self.state_time >= self.side_duration:
                    self.sub_state = "TURN"
                    self.state_time = 0.0
                    self.get_logger().info("Hình vuông: Chuyển sang trạng thái RẼ (TURN)")
            elif self.sub_state == "TURN":
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                if self.state_time >= self.turn_duration:
                    self.sub_state = "FORWARD"
                    self.state_time = 0.0
                    self.get_logger().info("Hình vuông: Chuyển sang trạng thái ĐI THẲNG (FORWARD)")

        # 4. Chế độ chạy hình số 8 (Figure Eight)
        elif self.mode == 'figure_eight':
            self.state_time += self.control_period
            
            # Đi thẳng và bẻ lái theo cung tròn
            twist.linear.x = self.linear_speed
            twist.angular.z = self.eight_direction * self.angular_speed
            
            # Hết chu kỳ nửa vòng tròn số 8, đổi hướng bẻ lái
            if self.state_time >= self.circle_duration:
                self.eight_direction *= -1.0 # Đảo hướng quay (Trái <-> Phải)
                self.state_time = 0.0
                dir_str = "TRÁI" if self.eight_direction > 0 else "PHẢI"
                self.get_logger().info(f"Số 8: Đổi hướng bẻ lái sang bên {dir_str}")

        else:
            self.get_logger().warn(f"Chế độ [{self.mode}] không được hỗ trợ!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Gửi lệnh
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Đã dừng xe an toàn.")
        # Thêm sleep ngắn để DDS kịp đẩy gói tin dừng xe đi trước khi Shutdown
        import time
        time.sleep(0.5)

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
