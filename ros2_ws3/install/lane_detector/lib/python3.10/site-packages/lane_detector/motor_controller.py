#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_sub')
        self.sub_= self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback,10)
       


    def twist_callback(self,msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Do something with the received values
        print('Linear X:', linear_x)
        print('Angular Z:', angular_z)

def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()