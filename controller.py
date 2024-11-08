import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from sensors.lidar import Lidar
from .flight_manager import FlightManager
from cv_bridge import CvBridge

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.image_sub = self.create_subscription(Image, '/uav1/camera/image_raw', self.image_callback, 10)
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')

        self.current_state = State()
        self.pose = PoseStamped()
        self.bridge = CvBridge()
        self.is_flying = False
        self.altitude = 2.0
        self.lidar = Lidar(self)
        self.flight_manager = FlightManager(self)

        self.drone_speed = 0.2
        self.drone_turn_speed = 0.1
        self.obstacle_avoidance_distance = 1.0
        self.obstacle_avoidance_duration = 2.0
        self.current_image = None
        self.timer = self.create_timer(0.1, self.fly)

    def state_cb(self, msg):
        self.current_state = msg

    def set_offboard_mode(self):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info("OFFBOARD mode set" if f.result().mode_sent else "Failed to set OFFBOARD mode"))

    def arm_drone(self):
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info("Drone armed" if f.result().success else "Failed to arm drone"))

    def fly(self):
        if not self.current_state.connected:
            self.get_logger().info("Waiting for connection to the drone...")
            return
        if self.current_state.mode != "OFFBOARD":
            self.set_offboard_mode()
        if not self.current_state.armed:
            self.arm_drone()

        if not self.is_flying:
            self.flight_manager.take_off(self.pose, self.altitude)
            self.is_flying = True
        elif self.current_image is not None:
            contours = self.detect_track(self.current_image)
            obstacles, gates = self.detect_obstacles_and_gates(self.current_image, contours)
            self.control_drone(contours, obstacles, gates)

        self.local_pos_pub.publish(self.pose)

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_track(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def detect_obstacles_and_gates(self, image, contours):
        obstacles = []
        gates = []
        for contour in contours:
            if self.is_gate(contour, image):
                gates.append(contour)
            else:
                obstacles.append(contour)
        return obstacles, gates

    def is_gate(self, contour, image):
        x, y, w, h = cv2.boundingRect(contour)
        if w > h:
            center_color = image[y + h // 2, x + w // 2]
            if self.is_green(center_color):
                return True
            elif self.is_red(center_color):
                self.pose.pose.orientation.z += np.pi
        return False

    def is_green(self, color):
        hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)
        return (36 <= hsv_color[0][0][0] <= 86) and (100 <= hsv_color[0][0][1] <= 255)

    def is_red(self, color):
        hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)
        return (0 <= hsv_color[0][0][0] <= 10) or (170 <= hsv_color[0][0][0] <= 180)

    def control_drone(self, contours, obstacles, gates):
        if not contours:
            self.pose.pose.orientation.z += self.drone_turn_speed
        else:
            for contour in contours:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    if cx < image.shape[1] // 3:
                        self.pose.pose.orientation.z -= self.drone_turn_speed
                    elif cx > 2 * image.shape[1] // 3:
                        self.pose.pose.orientation.z += self.drone_turn_speed
                    else:
                        self.pose.pose.position.x += self.drone_speed

        for obstacle in obstacles:
            if self.lidar.is_obstacle_ahead():
                self.pose.pose.position.y += self.drone_turn_speed
                self.pose.pose.position.x += self.drone_speed * self.obstacle_avoidance_duration
