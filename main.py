import rclpy
from drone_controller.controller import DroneController

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
