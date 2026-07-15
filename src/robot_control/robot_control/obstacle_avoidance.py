#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Publisher gửi tốc độ động cơ đến cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber nhận dữ liệu Lidar
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Khai báo các tham số cấu hình linh hoạt (Parameters)
        self.declare_parameter('obstacle_threshold', 0.6)  # Khoảng cách phát hiện vật cản trước mặt (mét)
        self.declare_parameter('linear_speed', 0.25)        # Vận tốc đi thẳng (m/s)
        self.declare_parameter('angular_speed', 0.5)       # Vận tốc xoay tránh vật cản (rad/s)
        self.declare_parameter('rotate_duration', 1.8)     # Thời gian xoay tránh vật cản (giây)
        self.declare_parameter('reverse_duration', 1.5)    # Thời gian lùi khi bị kẹt (giây)
        
        # Đọc tham số ban đầu
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.rotate_duration = self.get_parameter('rotate_duration').value
        self.reverse_duration = self.get_parameter('reverse_duration').value
        
        # Biến trạng thái robot (State Machine)
        self.state = "FORWARD"
        self.state_timer = 0.0
        
        # Bộ đệm dữ liệu cảm biến
        self.dist_front = float('inf')
        self.dist_left = float('inf')
        self.dist_right = float('inf')
        
        # Thiết lập timer chạy tính toán ở tần số 10Hz
        self.control_period = 0.1
        self.create_timer(self.control_period, self.control_loop)
        
        self.get_logger().info("Node Tự hành Tránh vật cản (Obstacle Avoidance) khởi động thành công.")
        self.get_logger().info(f"Cấu hình: Ngưỡng cản={self.obstacle_threshold}m, Tốc độ={self.linear_speed}m/s")

    def scan_callback(self, msg):
        """Xử lý và phân vùng mảng quét của cảm biến LaserScan"""
        # Đọc cấu hình góc quét
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        valid_count = 0
        inf_count = 0
        
        # Chuyển đổi góc sang radian cho các phân vùng
        # Front: -25° đến +25°
        front_min_rad, front_max_rad = np.radians(-25), np.radians(25)
        # Left: +25° đến +80°
        left_min_rad, left_max_rad = np.radians(25), np.radians(80)
        # Right: -80° đến -25°
        right_min_rad, right_max_rad = np.radians(-80), np.radians(-25)
        
        for i, r in enumerate(msg.ranges):
            # Loại bỏ các giá trị nhiễu hoặc ngoài giới hạn đo của cảm biến
            if r < msg.range_min or r > msg.range_max or np.isnan(r) or np.isinf(r):
                inf_count += 1
                continue
            
            valid_count += 1
            
            # Tính góc của tia quét thứ i
            angle = angle_min + i * angle_increment
            
            # Chuẩn hóa góc về khoảng [-pi, pi]
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            
            # Phân loại khoảng cách vào từng vùng quét tương ứng
            if front_min_rad <= angle <= front_max_rad:
                front_ranges.append(r)
            elif left_min_rad <= angle <= left_max_rad:
                left_ranges.append(r)
            elif right_min_rad <= angle <= right_max_rad:
                right_ranges.append(r)
                
        # Tính khoảng cách nhỏ nhất ở mỗi phân vùng (nếu vùng đó rỗng, mặc định là vô cùng lớn)
        self.dist_front = min(front_ranges) if front_ranges else float('inf')
        self.dist_left = min(left_ranges) if left_ranges else float('inf')
        self.dist_right = min(right_ranges) if right_ranges else float('inf')

        # Log debug định kỳ mỗi 50 lần nhận scan (~5 giây) để kiểm tra sức khỏe cảm biến
        if not hasattr(self, '_scan_debug_count'):
            self._scan_debug_count = 0
        self._scan_debug_count += 1
        if self._scan_debug_count % 50 == 1:
            # In thông tin góc quét
            self.get_logger().info(
                f"[SCAN DEBUG] angle_min={np.degrees(msg.angle_min):.1f}°, "
                f"angle_max={np.degrees(msg.angle_max):.1f}°, "
                f"increment={np.degrees(msg.angle_increment):.2f}°"
            )
            # In mẫu raw values ở giữa mảng (phía trước robot)
            center = len(msg.ranges) // 2
            raw_sample = [f"{msg.ranges[i]:.2f}" for i in range(max(0,center-5), min(len(msg.ranges),center+5))]
            self.get_logger().info(
                f"[SCAN DEBUG] Tổng tia={len(msg.ranges)}, Hợp lệ={valid_count}, "
                f"Vô cực/NaN={inf_count}, range_min={msg.range_min:.2f}, range_max={msg.range_max:.2f}"
            )
            self.get_logger().info(
                f"[SCAN DEBUG] Raw center[{center-5}:{center+5}]=[{', '.join(raw_sample)}]"
            )
            self.get_logger().info(
                f"[SCAN DEBUG] Front(n={len(front_ranges)})={self.dist_front:.2f}m, "
                f"Left(n={len(left_ranges)})={self.dist_left:.2f}m, "
                f"Right(n={len(right_ranges)})={self.dist_right:.2f}m"
            )

    def control_loop(self):
        """Hàm vòng lặp kiểm soát xe dựa trên máy trạng thái (FSM)"""
        # Cập nhật parameters trực tiếp
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.rotate_duration = self.get_parameter('rotate_duration').value
        self.reverse_duration = self.get_parameter('reverse_duration').value
        
        twist = Twist()
        
        # MÁY TRẠNG THÁI (STATE MACHINE)
        if self.state == "FORWARD":
            # Đi thẳng bình thường
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
            # Nếu phát hiện vật cản trước mặt thấp hơn ngưỡng an toàn
            if self.dist_front < self.obstacle_threshold:
                self.state = "STOP"
                self.state_timer = 0.0
                self.get_logger().info(f"Phát hiện vật cản trước mặt ({self.dist_front:.2f}m) -> DỪNG XE.")

        elif self.state == "STOP":
            # Dừng xe khẩn cấp và đưa ra quyết định rẽ hoặc lùi
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Đợi xe dừng hẳn 0.5s để chống rung lắc, sau đó kiểm tra không gian
            self.state_timer += self.control_period
            if self.state_timer >= 0.5:
                self.state_timer = 0.0
                self.get_logger().info(f"Khoảng không quét được: Trái={self.dist_left:.2f}m, Phải={self.dist_right:.2f}m")
                
                # Trường hợp bị kẹt cả 3 phía (Trái, Phải, Trước đều nhỏ hơn ngưỡng)
                if self.dist_left < self.obstacle_threshold and self.dist_right < self.obstacle_threshold:
                    self.state = "REVERSE"
                    self.get_logger().info("Cả hai hướng đều bị chặn -> Tiến hành LÙI XE.")
                # Lựa chọn bên có khoảng trống rộng hơn để rẽ
                elif self.dist_left >= self.dist_right:
                    self.state = "ROTATE_LEFT"
                    self.get_logger().info("Phía Trái rộng hơn -> Rẽ TRÁI.")
                else:
                    self.state = "ROTATE_RIGHT"
                    self.get_logger().info("Phía Phải rộng hơn -> Rẽ PHẢI.")

        elif self.state == "ROTATE_LEFT":
            # Xoay trái tại chỗ
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            
            self.state_timer += self.control_period
            if self.state_timer >= self.rotate_duration:
                self.state = "FORWARD"
                self.state_timer = 0.0
                self.get_logger().info("Hoàn thành rẽ trái -> Tiếp tục ĐI THẲNG.")

        elif self.state == "ROTATE_RIGHT":
            # Xoay phải tại chỗ
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
            
            self.state_timer += self.control_period
            if self.state_timer >= self.rotate_duration:
                self.state = "FORWARD"
                self.state_timer = 0.0
                self.get_logger().info("Hoàn thành rẽ phải -> Tiếp tục ĐI THẲNG.")

        elif self.state == "REVERSE":
            # Lùi xe phía sau để thoát hiểm
            twist.linear.x = -self.linear_speed
            twist.angular.z = 0.0
            
            self.state_timer += self.control_period
            if self.state_timer >= self.reverse_duration:
                # Sau khi lùi xong, chuyển sang quay trái để tìm hướng mới
                self.state = "ROTATE_LEFT"
                self.state_timer = 0.0
                self.get_logger().info("Đã lùi xe xong -> Chuyển sang xoay trái để thoát kẹt.")
                
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Gửi lệnh dừng xe nhiều lần liên tiếp để đảm bảo DDS truyền tải thành công
        trước khi node bị hủy. Nếu chỉ gửi 1 lần, gói tin rất dễ bị mất khi
        context đang shutdown, khiến robot tiếp tục bay với vận tốc cuối cùng
        → tọa độ NaN → Gazebo GUI crash."""
        import time
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Đang gửi lệnh dừng xe khẩn cấp...")
        for i in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.get_logger().info("Đã dừng xe tự hành an toàn.")

def main(args=None):
    # Tắt signal handler mặc định của rclpy để Ctrl+C không tự động shutdown context
    # trước khi ta kịp gửi lệnh dừng xe
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleAvoidanceNode()
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
