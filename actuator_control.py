import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LinearActuatorControlNode(Node):
    def __init__(self):
        super().__init__('linear_actuator_control_node')
        
        self.create_subscription(Bool, 'stop_actuator', self.stop_actuator_callback, 10)

    def stop_actuator_callback(self, msg):
        if not msg.data:
            self.get_logger().info('Actuator On')

def main(args=None):
    rclpy.init(args=args)
    linear_actuator_control_node = LinearActuatorControlNode()

    rclpy.spin(linear_actuator_control_node)
    linear_actuator_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
