#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import lcm
from bumblee import arduino_in, arduino_out
from random import randint

import time


class ArduinoManager():
    def __init__(self):
        rospy.init_node('arduino_listener_bumblebee', anonymous=True)
        self.lc = lcm.LCM()
        rospy.Subscriber("Arduino", String, self.sender_callback)

    def update(self):
        try:
            self.lc.handle()
        except Exception:
            self.shutdown()
            exit()

    def sender_callback(self, data):
        # try:
        received = data.data
        temp = received.split(',')
        #print (temp)
        dataOut = arduino_in()

        # temp now holds all the data
        #print (temp)
        dataOut.extreme_left = int(temp[0])
        dataOut.left = int(temp[1])
        dataOut.center = int(temp[2])
        dataOut.right = int(temp[3])
        dataOut.extreme_right = int(temp[4])

        dataOut.encoder_left = long(temp[5])
        dataOut.encoder_right = long(temp[6])

        dataOut.motorEnable = int(temp[7])

        dataOut.extra_io1 = int(temp[8])
        dataOut.extra_io2 = int(temp[9])

        dataOut.distance = int(temp[10])
        dataOut.status = int(temp[11])

        self.lc.publish("Arduino_Out", dataOut.encode())

        # except Exception:
        #    print ('Error in Callback')

    def spin(self):
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            # self.update()
            time.sleep(0.01)
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Node is shutting down")


def run():
    arduino = ArduinoManager()
    arduino.spin()


if __name__ == '__main__':
    print('Arduino Command Center - Listener Node Launched')
    try:
        run()
    except KeyboardInterrupt:
        exit()
