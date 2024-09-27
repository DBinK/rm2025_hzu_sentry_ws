#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rm_interfaces.msg import NavigationMsg

class CmdVel2Serial(Node):
    def __init__(self):
        super().__init__('cmd_vel2serial')

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publisher for the merged NavigationMsg
        self.publisher_ = self.create_publisher(NavigationMsg, '/nav/control', 10)

    def cmd_vel_callback(self, msg):
        # Process received Twist message and convert it to your control commands
        nav_msg = NavigationMsg()
        nav_msg.linear_velocity_x = msg.linear.x
        nav_msg.linear_velocity_y = msg.linear.y
        nav_msg.angular_velocity_z = msg.angular.z
        
        # Publish the navigation message
        self.publisher_.publish(nav_msg)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel2serial = CmdVel2Serial()

    rclpy.spin(cmd_vel2serial)

    # Shutdown
    cmd_vel2serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()