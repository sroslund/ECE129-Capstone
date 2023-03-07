#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from collections import deque
import numpy as np
import math

lin_acceleration = positionX = positionY = current_speed = 0
roll = pitch = yaw = angular_velocity = 0
steering_error_buffer = deque(maxlen=10)
#parameters
KPv = 6
KDv = 0.2
KIv = 0.025
Kps = 2
Kds = 0.02


def PID_VelocityControl(target_speed, current_speed):
    error_buffer = deque(maxlen=10)
    error = target_speed - current_speed
    error_buffer.append(error)
    if len(error_buffer) >= 2:
        _ie = sum(error_buffer) * 1/100 
    else:
        _ie = 0.0
    return np.clip((KPv * error) + (KDv * lin_acceleration) + (KIv * _ie), -1, 1)


def PD_Steering_Control():
    global steering_error_buffer, yaw
    alpha = math.atan2(2 - positionY, -1 - positionX) - yaw 
    desired_yaw = math.atan2(2*math.sin(alpha), 1)
    error = desired_yaw - yaw
    if (error > math.pi):
        error -= 2*math.pi
    if (error < -math.pi):
        error += 2*math.pi
    steering_error_buffer.append(error)
    yaw_velocity = (Kps * error) + (Kds * angular_velocity)
    return yaw_velocity


def callback(data):
    global roll, pitch, yaw, positionX, positionY, current_speed
    print(data.pose.pose.position)
    positionX = data.pose.pose.position.x
    positionY = data.pose.pose.position.y
    current_speed = data.twist.twist.linear.x
    q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)    


def callback2(data):
    global lin_acceleration, angular_velocity
    lin_acceleration = data.linear_acceleration.x
    angular_velocity = data.angular_velocity.z


def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.Subscriber("imu", Imu, callback2)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        #linear = Vector3(PID_VelocityControl(0.3,current_speed),0,0)
        linear = Vector3(0.1,0,0)
        angular = Vector3(0,0,PD_Steering_Control())
        test = Twist(linear,angular)
        #rospy.loginfo(test)
        pub.publish(test)
        rate.sleep()
    rospy.spin() 
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
