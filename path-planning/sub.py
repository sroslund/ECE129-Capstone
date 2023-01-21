#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def sub_callback(message):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)
    
def sub():

    rospy.init_node('sub', anonymous=True)

    rospy.Subscriber("pub", String, sub_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    sub()