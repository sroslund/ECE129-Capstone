#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import lcm
from bumblee import arduino_out
from random import randint

import time


class ArduinoManager():
    def __init__(self):

        rospy.init_node('arduino_sender_bumblebee', anonymous=True)

        self.lc = lcm.LCM()

        self.pub = rospy.Publisher('BumbleBee_Receiver', String, queue_size=5)

        self.lc.subscribe("Arduino_In", self.arduino_sendout)

    def update(self):

        try:
            self.lc.handle()
        except Exception:
            self.shutdown()
            exit()

    def arduino_sendout(self, channel, data):

        dataIn = arduino_out.decode(data)

        rand = randint(0, 100)

        Data_toPack = str(rand) + ',' + str(dataIn.leftspeed) + ',' + str(dataIn.rightspeed) + ',' + str(dataIn.motorEnable) + \
            ',' + str(dataIn.io_device1) + ',' + str(dataIn.io_device2) + \
            ',' + str(dataIn.io_device3) + ',' + str(rand)

        self.pub.publish(Data_toPack)

    def spin(self):
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.update()
            time.sleep(0.01)
        rospy.spin()

    def shutdown(self):

        rospy.loginfo("Node is shutting down")


def run():
    arduino = ArduinoManager()
    arduino.spin()


if __name__ == '__main__':
    print('Arduino Command Center - Sender Node Launched')
    try:
        run()
    except KeyboardInterrupt:
        exit()
