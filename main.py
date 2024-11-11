import rclpy
from drone_controller.controller import DroneController
from drone_controller.land import LandManager
from rclpy.node import Node

def main(args=None):
    # Инициализация rclpy
    rclpy.init(args=args)

    # Создание экземпляра DroneController
    drone_controller = DroneController()

    # Создание экземпляра Node для LandManager
    land_manager_node = Node("land_manager_node")
    land_manager = LandManager(land_manager_node)
    rclpy.spin(drone_controller)
    # Уничтожение узлов при выходе
    drone_controller.destroy_node()

if __name__ == '__main__':
    main()
