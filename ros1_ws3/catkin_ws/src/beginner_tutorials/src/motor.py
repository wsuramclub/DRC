#!/usr/bin/env python3.7
import rospy
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import board
import busio
import time
from approxeng.input.selectbinder import ControllerResource

i2c_bus1=(busio.I2C(board.SCL, board.SDA))
kit = ServoKit(channels=16, i2c=i2c_bus1)


def twist_callback(msg):
	rospy.loginfo('steering angle=%.2f',msg.angular.z)
	angular_z = msg.angular.z
	if angular_z>=45 and angular_z<=89:
		steering_angle=int(90+((angular_z-45)*90)/45)
	
	else:
		steering_angle=int(0+((angular_z-91)*90)/45)

	# #print("Initializing Servos")
	# i2c_bus1=(busio.I2C(board.SCL, board.SDA))
	# #print("Initializing ServoKit")
	# kit = ServoKit(channels=16, i2c=i2c_bus1)
	# # kit[0] is the bottom servo
	# # kit[1] is the top servo
	# #print("Done initializing")
	# sweep = range(0,180)
	# # for degree in sweep :
	# 	#kit.servo[0].angle=degree
	# 	# kit.servo[1].angle=degree
	# 	#	 time.sleep(0.01)
	
	# sweep = range(180,0, -1)
	#for degree in sweep :
		#kit.servo[0].angle=degree

	# cat -A [filename] | tail -10   
	kit.continuous_servo[0].throttle = 0.2
	kit.servo[1].angle = steering_angle
	time.sleep(0.5)


def twist_publisher():
	
	
	print("Initializing ServoKit")
	
	rospy.init_node('twist_publisher', anonymous=True)
	rospy.Subscriber('twist_topic', Twist,  twist_callback)
	rospy.spin()
    

if __name__ == '__main__':
	twist_publisher()
    
