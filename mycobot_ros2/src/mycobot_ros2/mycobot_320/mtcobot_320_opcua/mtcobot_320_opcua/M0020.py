import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

# M0001 센서 상태를 수신하고 메시지를 출력하는 노드
class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        self.get_logger().info('M0001 센서 리스너 노드 초기화. plc/sensor_status 토픽 구독 시작.')
        
        # 'plc/sensor_status' 토픽 구독 설정
        self.subscription = self.create_subscription(
            Int16,
            'plc/sensor_status',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        """plc/sensor_status 토픽으로부터 메시지를 수신할 때 호출되는 콜백 함수"""
        sensor_value = msg.data
        
        # M0001 값이 1일 때 (센서가 활성화되었을 때) "Hello~" 출력
        if sensor_value == 1:
            self.get_logger().info('M0001 센서 활성화 감지: Hello~')
        # 값이 0일 때 (선택 사항: 센서 비활성화 시 메시지)
        elif sensor_value == 0:
            # self.get_logger().info('M0001 센서 비활성화.')
            pass # 0일 때는 아무것도 출력하지 않도록 설정
        else:
            self.get_logger().warn(f'M0001에서 예상치 못한 값 수신: {sensor_value}')

def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()
    
    try:
        rclpy.spin(sensor_listener)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()