#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def publish():
    pub = rospy.Publisher('pub', String, queue_size=10)
 
    rospy.init_node('pub', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(1) # 1hz
    #keep publishing until a Ctrl-C is pressed
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
