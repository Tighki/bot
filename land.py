from drone_controller.utilities import log_info
import rclpy

class LandManager:
    def __init__(self, node):
        self.node = node
        self.reached_altitude = False

    def take_off(self, pose, target_altitude):
        if not self.reached_altitude:
            if abs(pose.pose.position.z - target_altitude) > 0.05:
                # pose.pose.position.z -= 0.5  
                pose.pose.position.x -= -2.2
                pose.pose.position.y -= -15.2
                log_info(self.node, f"Land: {pose.pose.position.z:.2f}m")
            else:
                self.reached_altitude = True
                log_info(self.node, "Land movement.")


    def explore_environment(self, pose, lidar):
        if self.reached_altitude:
            if lidar.is_obstacle_ahead():
                self.avoid_obstacle(pose)
            else:
                if abs(pose.pose.position.x) < 15 and abs(pose.pose.position.y) < 15:
                    pose.pose.position.x += 0.05 if abs(pose.pose.position.x) < 15 else 0.0
                    pose.pose.position.y += 0.05 if abs(pose.pose.position.y) >= 15 else 0.0
                else:
                    pose.pose.position.x, pose.pose.position.y = -15, -15

    def avoid_obstacle(self, pose):
        angle_change = 0.2
        pose.pose.orientation.z = (pose.pose.orientation.z + angle_change) % (2 * 3.14159)
        log_info(self.node, "Avoiding obstacle")
