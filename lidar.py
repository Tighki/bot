from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self, node):
        self.node = node
        self.safe_distance = 2.0
        self.laser_data = []
        self.node.create_subscription(LaserScan, '/uav1/laser/scan', self.laser_callback, 10)

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def is_obstacle_ahead(self):
        filtered_data = [d for d in self.laser_data if d > 0]
        return any(distance < self.safe_distance for distance in filtered_data)