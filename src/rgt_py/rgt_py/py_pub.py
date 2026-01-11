import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TempSensorSim(Node):
    def __init__(self):
        super().__init__('temp_sensor_sim')
        self.publisher_ = self.create_publisher(Float64, 'raw_temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1초 주기로 자동 발행
        self.data_list = [23.5, 24.1, 23.8, 25.2, 24.7, 26.1]
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.data_list):
            msg = Float64()
            msg.data = self.data_list[self.index]
            self.publisher_.publish(msg)
            self.get_logger().info(f'발행된 센서 데이터: {msg.data}')
            self.index += 1
        else:
            self.get_logger().info('모든 테스트 데이터 발행 완료.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = TempSensorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()