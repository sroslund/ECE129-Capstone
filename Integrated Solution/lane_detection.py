#!/usr/bin/python3

'''
vela, scott, robert
'''

import math
import datetime
import sys
import cv2
import numpy as np
import logging


class lane_follower(object):
    def __init__(self, car=None):
        logging.info('lane_follower')
        self.curr_steering_angle = 0

    def follow_lane(self, frame):

        lane_segments, frame = detect_lane(frame)
        follow_frame = self.steer(frame, lane_segments)

        return follow_frame

    def steer(self, frame, lane_segments):
        if len(lane_segments) == 0:
            return frame
        new_steering_angle = compute_steering_angle(frame, lane_segments)
        return curr_heading_image


def detect_lane(frame):

    edges = detect_edges(frame)

    line_segments = detect_line_segments(edges)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)

    return lane_lines, lane_lines_image


def detect_edges(frame):
    # filter for blue lane lines HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([81, 81, 0])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # detect edges
    edges = cv2.Canny(mask, 200, 400)
    return edges


def detect_line_segments(edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1
    angle = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)
    return line_segments


def average_slope_intercept(frame, line_segments):

    lane_lines = []
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    # left lane line segment should be on left 2/3 of the screen
    left_region_boundary = width * (1 - boundary)
    # right lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info(
                    'skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)

    return lane_lines


def compute_steering_angle(frame, lane_lines):

    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug(
            'Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        camera_mid_offset_percent = 0.02
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    # angle (in radian) to center vertical line
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    # angle (in degrees) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    # this is the steering angle
    steering_angle = angle_to_mid_deg + 90

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle

############################
# Utility Functions


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2),
                         line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]
