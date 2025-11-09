import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16 
from pymodbus.client.sync import ModbusSerialClient
from asyncua import Server, ua
import asyncio
import threading
import time
import queue  # ⭐ 큐 모듈 추가

# --- Configuration (PLC와 일치하는지 재확인하세요!) ---
COM_PORT = "/dev/ttyUSB0"
MODBUS_BAUDRATE = 9600
MODBUS_UNIT_ID = 3
TIMEOUT_SECONDS = 1 

# Modbus Addresses (Offset)
M0010_SENSOR_OFFSET = 16
M0020_COIL_OFFSET = 32

OPCUA_ENDPOINT = "opc.tcp://0.0.0.0:4840/ros2_plc_gateway/"
OPCUA_NAMESPACE_URI = "http://yourcompany.com/plc_ros_control"

# -----------------------------------------------------

class PlcIoGatewayNode(Node):
    def __init__(self):
        super().__init__('plc_io_gateway')
        self.get_logger().info('Initializing PLC I/O Gateway Node with Threading.')

        # 1. 큐 초기화 (스레드 간 안전한 통신을 위해)
        self.modbus_write_queue = queue.Queue() 
        self.modbus_read_data = {'sensor': 0, 'coil': 0} 
        self.read_data_lock = threading.Lock() 

        # 2. Modbus 클라이언트 초기화
        self.modbus_client = None
        self.modbus_connected = False
        
        # 3. ROS 2 퍼블리셔 및 서브스크라이버
        self.sensor_pub = self.create_publisher(Int16, 'plc/sensor_status', 10)
        self.coil_status_pub = self.create_publisher(Int16, 'plc/coil_status', 10)
        self.coil_cmd_sub = self.create_subscription(
            Int16, 'plc/coil_command', self.coil_command_callback, 10
        )

        # 4. OPC UA 서버 및 노드 변수 초기화
        self.opcua_server = None
        self.sensor_opcua_node = None 
        self.coil_opcua_node = None 
        self.opcua_loop = None 
        
        # 5. OPC UA 서버 스레드 시작
        self.opcua_thread = threading.Thread(target=self._run_opcua_server_in_thread, daemon=True)
        self.opcua_thread.start()
        
        # 6. Modbus Worker 스레드 시작
        self.modbus_thread = threading.Thread(target=self._modbus_worker, daemon=True)
        self.modbus_thread.start()
        
        # 7. 타이머 콜백 (Modbus Worker에서 읽은 값을 ROS/OPC UA로 발행)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self._gateway_publish_callback)

    # -----------------------------------------------------
    # 🔴 OPC UA 서버 비동기 로직 (OPC UA 스레드)
    # -----------------------------------------------------
    def _run_opcua_server_in_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.opcua_loop = loop 
        loop.run_until_complete(self._setup_opcua_server())
        loop.run_forever()

    async def _setup_opcua_server(self):
        self.opcua_server = Server()
        await self.opcua_server.init()
        self.opcua_server.set_endpoint(OPCUA_ENDPOINT)
        self.opcua_server.set_server_name("ROS2 PLC Gateway")

        idx = await self.opcua_server.register_namespace(OPCUA_NAMESPACE_URI)
        plc_folder = await self.opcua_server.nodes.objects.add_object(idx, 'PLC_IO_Data')
        
        # M0010 → 센서, M0020 → 코일
        self.sensor_opcua_node = await plc_folder.add_variable(
            idx, 'M0010_Sensor_Input', 0, varianttype=ua.VariantType.Int16
        )
        self.coil_opcua_node = await plc_folder.add_variable(
            idx, 'M0020_Coil_Output', 0, varianttype=ua.VariantType.Int16
        )
        
        await self.opcua_server.start()
        self.get_logger().info('✅ OPC UA Server running in thread.')

    # -----------------------------------------------------
    # ⭐ Modbus Worker 스레드 (Modbus 통신 전담)
    # -----------------------------------------------------
    def _modbus_worker(self):
        self.modbus_client = ModbusSerialClient(
            method="rtu", port=COM_PORT, baudrate=MODBUS_BAUDRATE,
            parity='N', stopbits=1, bytesize=8, timeout=TIMEOUT_SECONDS
        )
        
        while rclpy.ok():
            if not self.modbus_connected:
                if self.modbus_client.connect():
                    self.modbus_connected = True
                    self.get_logger().info('✅ Modbus 연결 성공 (Worker Thread)!')
                else:
                    time.sleep(2)
                    continue
            
            try:
                # 1. 쓰기 명령 처리
                try:
                    offset, value = self.modbus_write_queue.get(timeout=0.1) 
                    self.modbus_client.write_coil(offset, value, unit=MODBUS_UNIT_ID)
                    self.get_logger().info(f"✅ Modbus Write Coil ({offset}) Success from ROS command.")
                    self.modbus_write_queue.task_done()
                except queue.Empty:
                    pass 
                
                # 2. 읽기 (Modbus Read)
                rr_sensor = self.modbus_client.read_coils(M0010_SENSOR_OFFSET, 1, unit=MODBUS_UNIT_ID)
                rr_coil = self.modbus_client.read_coils(M0020_COIL_OFFSET, 1, unit=MODBUS_UNIT_ID)
                
                with self.read_data_lock:
                    if hasattr(rr_sensor, 'bits') and rr_sensor.bits:
                        self.modbus_read_data['sensor'] = int(rr_sensor.bits[0])
                    if hasattr(rr_coil, 'bits') and rr_coil.bits:
                        self.modbus_read_data['coil'] = int(rr_coil.bits[0])

            except Exception as e:
                self.get_logger().error(f'❌ Modbus 통신 예외 발생 (Worker Thread): {e}')
                self.modbus_client.close()
                self.modbus_connected = False
            
            time.sleep(0.05)

    # -----------------------------------------------------
    # 🔵 ROS 2 Publish & OPC UA Update
    # -----------------------------------------------------
    def _gateway_publish_callback(self):
        with self.read_data_lock:
            sensor_val = self.modbus_read_data['sensor']
            coil_val = self.modbus_read_data['coil']

        self.sensor_pub.publish(Int16(data=sensor_val))
        self.coil_status_pub.publish(Int16(data=coil_val))

        if self.opcua_loop and self.sensor_opcua_node:
            asyncio.run_coroutine_threadsafe(
                self.sensor_opcua_node.write_value(ua.Variant(sensor_val, ua.VariantType.Int16)), self.opcua_loop
            )
        if self.opcua_loop and self.coil_opcua_node:
            asyncio.run_coroutine_threadsafe(
                self.coil_opcua_node.write_value(ua.Variant(coil_val, ua.VariantType.Int16)), self.opcua_loop
            )

    # -----------------------------------------------------
    # 🟢 ROS 2 명령 수신 및 Modbus 쓰기
    # -----------------------------------------------------
    def coil_command_callback(self, msg):
        cmd_val = msg.data
        self.modbus_write_queue.put((M0020_COIL_OFFSET, bool(cmd_val))) 
        self.get_logger().info(f'ROS Command Received: Sending Coil M0020 command to Worker.')

    # -----------------------------------------------------
    # 🛑 Node Shutdown
    # -----------------------------------------------------
    def destroy_node(self):
        if self.opcua_server and self.opcua_loop: 
            asyncio.run_coroutine_threadsafe(self.opcua_server.stop(), self.opcua_loop)
            self.opcua_loop.stop()
            self.opcua_thread.join(timeout=2)
            self.get_logger().info('👋 OPC UA Server stopped.')

        if self.modbus_client:
            self.modbus_client.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gateway_node = PlcIoGatewayNode()
    try:
        rclpy.spin(gateway_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        gateway_node.get_logger().error(f"Critical error: {e}")
    finally:
        gateway_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
