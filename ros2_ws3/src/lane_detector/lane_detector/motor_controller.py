#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import board
import busio
import time
from approxeng.input.selectbinder import ControllerResource

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

        print("Initializing Servos")
        i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        print("Initializing ServoKit")
        kit = ServoKit(channels=16, i2c=i2c_bus0)
        # kit[0] is the bottom servo
        # kit[1] is the top servo
        print("Done initializing")
        sweep = range(0,180)
        for degree in sweep :
            kit.servo[0].angle=degree
            # kit.servo[1].angle=degree
            # time.sleep(0.01)

        time.sleep(0.5)
        sweep = range(180,0, -1)
        for degree in sweep :
            kit.servo[0].angle=degree

        # cat -A [filename] | tail -10   
        last_presses = None

        kit.servo[0].angle = angular_z
        kit.continuous_servo[1].throttle = linear_x

def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()