#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from collections import deque
import numpy as np
import math

#parameters
# KPv = 6
# KDv = 0.2
# KIv = 0.025


# def PID_VelocityControl(target_speed, current_speed, dt):
#     error_buffer = deque(maxlen=10)
#     error = target_speed - current_speed
#     error_buffer.append(error)

#     if len(error_buffer) >= 2:
#         _de = (error_buffer[-1] - error_buffer[-2]) / dt 
#         _ie = sum(error_buffer) * dt 
#     else:
#         _de = 0.0
#         _ie = 0.0

#     return np.clip((KPv * error) + (KDv * _de) + (KIv * _ie), -1, 1)

# def PD_Steering_Control(data, trajectory, point_index):
#     alpha = math.atan2(6 - data.pose.pose.position.y, 5 - data.pose.pose.position.x) #- yaw 

def callback(data):

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    #print(data.pose.pose.orientation)
    q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    print("Angle: " + str(math.degrees(yaw)))
    linear = Vector3(0,0,0)
    angular = Vector3(0,0,.5)
    test = Twist(linear,angular)

    #rospy.loginfo(test)
    pub.publish(test)
    rate.sleep()

def talker():
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin() 
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
