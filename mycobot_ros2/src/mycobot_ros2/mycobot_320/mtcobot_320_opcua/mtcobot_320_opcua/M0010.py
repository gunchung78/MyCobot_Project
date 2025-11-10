import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

# M0010 코일 상태를 수신하고 메시지를 출력하는 노드
class CoilListener(Node):
    def __init__(self):
        super().__init__('coil_listener')
        self.get_logger().info('M0010 코일 리스너 노드 초기화. plc/coil_status 토픽 구독 시작.')

        # 'plc/coil_status' 토픽 구독 설정
        self.subscription = self.create_subscription(
            Int16,
            'plc/coil_status',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        """plc/coil_status 토픽으로부터 메시지를 수신할 때 호출되는 콜백 함수"""
        coil_value = msg.data
        
        # M0010 값이 1일 때 (코일이 ON 되었을 때) "Everyone~" 출력
        if coil_value == 1:
            self.get_logger().info('M0010 코일 활성화 감지: Everyone~')
        # 값이 0일 때 (선택 사항: 코일 OFF 시 메시지)
        elif coil_value == 0:
            # self.get_logger().info('M0010 코일 비활성화.')
            pass # 0일 때는 아무것도 출력하지 않도록 설정
        else:
            self.get_logger().warn(f'M0010에서 예상치 못한 값 수신: {coil_value}')

def main(args=None):
    rclpy.init(args=args)
    coil_listener = CoilListener()

    try:
        rclpy.spin(coil_listener)
    except KeyboardInterrupt:
        pass
    finally:
        coil_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()