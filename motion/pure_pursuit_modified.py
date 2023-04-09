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
import matplotlib.pyplot as plt

lin_acceleration = current_speed = 0
roll = pitch = yaw = angular_velocity = desired_yaw = 0
steering_error_buffer = deque(maxlen=10)
old_nearest_index = None
positionX = -3
positionY = 1
#parameters
KPv = 1#6
KDv = 0#0.2
KIv = 0#0.05
Kps = 2
Kds = 0.02

coordinatesX = [-3,-2,-1,0]
coordinatesY = [1,2,1.5,0]
#desiredX = -2
#desiredY = 2

def calc_distance(pointX, pointY):
    return np.hypot(positionX - pointX, positionY - pointY)

def search_desired_coordinates():
    global old_nearest_index
    if old_nearest_index is None:
        desiredX = [positionX - icx for icx in coordinatesX]
        desiredY = [positionY - icy for icy in coordinatesY]
        d = np.hypot(desiredX, desiredY)
        ind = np.argmin(d)
        old_nearest_index = ind
    else:
        ind = old_nearest_index
        distance_this_index = calc_distance(coordinatesX[ind],coordinatesY[ind])
        print(ind)
        while True:
            if (ind+1) >= len(coordinatesX):
                break
            distance_next_index = calc_distance(coordinatesX[ind+1],coordinatesY[ind+1])
            if distance_this_index < distance_next_index:
                break
            if (ind + 1) < len(coordinatesX):
                ind = ind + 1
            else:
                ind
            distance_this_index = distance_next_index
        old_nearest_index = ind
    if (ind+1) == len(coordinatesX):
        return [coordinatesX[ind],coordinatesY[ind]]
    else:
        return [coordinatesX[ind+1],coordinatesY[ind+1]]

def PID_VelocityControl(target_speed, current_speed):
    error_buffer = deque(maxlen=10)
    error = target_speed - current_speed
    error_buffer.append(error)
    if len(error_buffer) >= 2:
        _ie = sum(error_buffer) * 1/200
    else:
        _ie = 0.0
    return np.clip((KPv * error) + (KDv * lin_acceleration) + (KIv * _ie), -1, 1)


def PD_Steering_Control():
    global steering_error_buffer, yaw, desired_yaw
    desiredX,desiredY = search_desired_coordinates()
    alpha = math.atan2(desiredY - positionY, desiredX - positionX) - yaw 
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
    #print(data.pose.pose.position)
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
    f = open('test.txt', 'w')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.Subscriber("imu", Imu, callback2)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        linear = Vector3(PID_VelocityControl(0.2,current_speed),0,0)
        #linear = Vector3(0.1,0,0)
        angular = Vector3(0,0,PD_Steering_Control())
        f.write('\n' + str(yaw) + '\t' + str(desired_yaw))
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
