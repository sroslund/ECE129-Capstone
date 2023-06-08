#!/usr/bin/python3

'''
vela, scott, robert
'''
import cv2
import logging
import time
import datetime
from lane_detection import lane_follower


class Temujin(object):
    def __init__(self) -> None:
        logging.info('Start Temujin')
        logging.debug('Connect Camera')
        self.camera = cv2.VideoCapture(-1)

        logging.debug('Inport lane_follower')
        self.lane_follower = lane_follower(self)

        logging.info('Init Done')

    def auton(self):
        logging.info('Autonmous Driving')
        while self.camera.isOpened():
            ret, frame = self.camera.read()

            lane_frame = self.lane_follower.follow_lane(frame)
            cv2.imshow("Final Output", lane_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
                break


def main():
    with Temujin() as scott:
        scott.auton()


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG,
                        format='%(levelname)-5s:%(asctime)s: %(message)s')
    main()
