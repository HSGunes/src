import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        # This parameter can be adjusted in a config file or launch file
        self.declare_parameter('wheel_base', 0.5) # meters
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
            
        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10)
        
        self.get_logger().info('CmdVelConverter started. Publishing linear.x to /velocity_controller/commands and angular.z to /position_controller/commands')

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Publish linear velocity to velocity controller

        if  linear_x > 2.0:
            linear_x = 2.0
        elif linear_x < -2.0:
            linear_x = -2.0

        if angular_z > 0.7:
            angular_z = 0.7
        elif angular_z < -0.7:
            angular_z = -0.7

        vel_command = Float64MultiArray()
        vel_command.data = [linear_x, linear_x]
        self.velocity_publisher.publish(vel_command)

        # Publish angular velocity (or steering position) to position controller
        pos_command = Float64MultiArray()
        pos_command.data = [angular_z, angular_z]
        self.position_publisher.publish(pos_command)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverter()
    rclpy.spin(cmd_vel_converter)
    cmd_vel_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 