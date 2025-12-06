import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid controller node started')

def main(args=None):
    rclpy.init(args=args)
    humanoid_controller = HumanoidController()
    rclpy.spin(humanoid_controller)
    humanoid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
