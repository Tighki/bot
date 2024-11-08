import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensors.lidar import Lidar
from .flight_manager import FlightManager

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Подписки и публикации для MAVROS
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # Инициализация параметров и состояния дрона
        self.current_state = State()
        self.pose = PoseStamped()
        self.is_flying = False
        self.altitude = 2.0

        # Подключение датчиков и менеджера полета
        self.lidar = Lidar(self)
        self.flight_manager = FlightManager(self)

        # Таймер для вызова основной функции управления полетом
        self.timer = self.create_timer(0.1, self.fly)

    def state_cb(self, msg):
        self.current_state = msg

    def fly(self):
        # Проверка и установка режима OFFBOARD и вооружение дрона
        if not self.current_state.connected:
            self.get_logger().info("Waiting for connection to the drone...")
            return
        if self.current_state.mode != "OFFBOARD":
            self.set_offboard_mode()
        if not self.current_state.armed:
            self.arm_drone()

        # Выполнение взлета и полета
        if not self.is_flying:
            self.flight_manager.take_off(self.pose, self.altitude)
            self.is_flying = True
        else:
            self.flight_manager.explore_environment(self.pose, self.lidar)

        # Публикация позиции дрона
        self.local_pos_pub.publish(self.pose)

    def set_offboard_mode(self):
        # Установка режима OFFBOARD
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info("OFFBOARD mode set" if f.result().mode_sent else "Failed to set OFFBOARD mode"))

    def arm_drone(self):
        # Вооружение дрона
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info("Drone armed" if f.result().success else "Failed to arm drone"))