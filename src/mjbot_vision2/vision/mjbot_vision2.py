#!/usr/bin/env python3
import datetime as dt
import numpy as np
import cv2

# from vision.depth import *
from scripts.sort import Sort

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange

import csv
from scripts.yolo import *

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from example_interfaces.msg import Int32
from example_interfaces.msg import Int16MultiArray
import threading
from rclpy.executors import MultiThreadedExecutor

import os
import urllib
import traceback
import time
import sys

import cv2
from math import exp

w, h = 640, 360
CONFIDENCE_THRESHOLD = 0.25


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_process")

        self.fps = 0
        self.duration_sec = 0
        self.runtime_sec = 0

        self.owner_size = [0, 0]  # [w, h]
        self.owner_center = [0, 0]  # [cx, cy]
        self.owner_size_prev = [0, 0]  # for fall detection
        self.owner_size_diff = [0, 0]  # [w_diff, h_diff]
        self.owner_exists = False
        self.owner_fall = False

        self.publisher_owner_exists_ = self.create_publisher(
            Bool, "owner_exists", 10)
        self.publisher_owner_center_ = self.create_publisher(
            Int16MultiArray, "owner_center", 10)
        self.publisher_owner_size_ = self.create_publisher(
            Int16MultiArray, "owner_size", 10)
        self.publisher_owner_fall_ = self.create_publisher(
            Bool, "owner_fall", 10)
        self.get_logger().info("Node has been started")

    def publish_owner_exists(self):
        msg = Bool()
        msg.data = self.owner_exists
        self.publisher_owner_exists_.publish(msg)
        self.get_logger().info("PUB: /owner_fall: {}".format(msg.data))

    def publish_owner_size(self):
        msg = Int16MultiArray()
        msg.data = [self.owner_size[0], self.owner_size[1]]
        self.publisher_owner_size_.publish(msg)
        self.get_logger().info("PUB: /owner_size: {}".format(msg.data))

    def publish_owner_center(self):
        msg = Int16MultiArray()
        msg.data = [self.owner_center[0], self.owner_center[1]]
        self.publisher_owner_center_.publish(msg)
        self.get_logger().info("PUB: /owner_center: {}".format(msg.data))

    def publish_owner_fall(self):
        msg = Bool()
        msg.data = self.owner_fall
        self.publisher_owner_fall_.publish(msg)
        self.get_logger().info("PUB: /owner_fall: {}".format(msg.data))

    def init_video(self):
        self.cap = cv2.VideoCapture(1)
        # self.cap = cv2.VideoCapture('/home/drcl/ros2_ws/src/vision/vision/video/fall-01.mp4')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

        if not self.cap.isOpened():
            self.get_logger().info("VIDEO: Cannot open video")
        else:
            self.get_logger().info("VIDEO: Open video")

    def read_video(self):
        ret, src = self.cap.read()
        self.img = cv2.resize(src, dsize=(640, 360),)

        self.img = cv2.copyMakeBorder(
            self.img, 140, 140, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))

        return self.img

        # ret, src = self.cap.read()
        # self.img = cv2.resize(src, dsize=(0, 0), fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
        # self.img = cv2.copyMakeBorder(self.img, 80, 80, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))

    def init_sort(self):
        self.sort = Sort(max_age=2, min_hits=3, iou_threshold=0.3, )

    def run_sort(self, boxes, scores):
        if boxes is not None:
            scores = scores.reshape((-1, 1))
            dets = np.concatenate((boxes, scores), 1)
            tracker = self.sort.update(dets)
            for trk in tracker:
                trk = trk.astype(int)
                cv2.rectangle(
                    self.img, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 2)
                cv2.putText(
                    self.img,  "ID:"+str(trk[4]), (trk[0], trk[1] + 12), 1, 1, (255, 255, 255), 2)

    def detect_fall(self):
        # if object detection is failed
        if self.owner_size[0] == 0 and self.owner_size[1] == 0:
            self.owner_size[0] = self.owner_size_prev[0]
            self.owner_size[1] = self.owner_size_prev[1]
            self.owner_exists = False
        else:
            self.owner_exists = True

        self.owner_size_diff[0] = (
            self.owner_size[0] - self.owner_size_prev[0]) / self.duration_sec  # w_diff
        self.owner_size_diff[1] = (
            self.owner_size[1] - self.owner_size_prev[1]) / self.duration_sec  # h_diff

        if self.owner_size_diff[1] <= -500:
            self.owner_fall = True
        else:
            self.owner_fall = False

        self.owner_size_prev[0] = self.owner_size[0]
        self.owner_size_prev[1] = self.owner_size[1]

    def init_csv_log(self):
        self.fieldnames = ["Duration", "Runtime", "FPS",
                           "Width", "Height",
                           "Width Diff", "Height Diff",
                           "Owner Exists", "Fall Detection"]

        with open('/home/drcl/Desktop/mjbot_2023/src/mjbot_vision/vision/log.csv', 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()

    def run_csv_log(self):
        with open('/home/drcl/Desktop/mjbot_2023/src/mjbot_vision/vision/log.csv', 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)

            info = {
                "Duration": self.duration_sec,
                "Runtime": self.runtime_sec,
                "FPS": self.fps,

                "Width": self.owner_size[0],
                "Height": self.owner_size[1],

                "Width Diff": self.owner_size_diff[0],
                "Height Diff": self.owner_size_diff[1],

                "Owner Exists": self.owner_exists,
                "Fall Detection": self.owner_fall
            }

            csv_writer.writerow(info)

    # publish persion coordinates
    def publish_person_coordinates(self, results):
        # Check if a person is detected
        for box, label_idx, score in results:
            # Get the class label
            label = CLASSES[label_idx]

            # Check if the detected object is a "person" with a certain confidence threshold
            if label == "person" and score >= CONFIDENCE_THRESHOLD:
                # Get the coordinates of the person
                x1, y1, x2, y2 = [int(coord) for coord in box]

                # Calculate the center of the person
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2

                # Publish the coordinates
                msg = Int16MultiArray()
                msg.data = [x_center, y_center]

                # Publish owner_center
                self.publisher_owner_center_.publish(msg)
                self.get_logger().info("PUB: /owner_center: {}".format(msg.data))
            else:
                # publish previous coordinates
                msg = Int16MultiArray()
                msg.data = [x_center, y_center]
                # Publish owner_center
                self.publisher_owner_center_.publish(msg)
                self.get_logger().info("PUB: /owner_center: {}".format(msg.data))
        # return peson coordinates in boxes, scores

        return

    def peson_checker(self, results):
        # Check if a person is detected
        for box, label_idx, score in results:
            # Get the class label
            label = CLASSES[label_idx]

            # Check if the detected object is a "person" with a certain confidence threshold
            if label == "person" and score >= CONFIDENCE_THRESHOLD:
                # Get the coordinates of the person
                return box, label_idx, score
            else:
                # return previous coordinates
                return box, label_idx, score


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    node.init_video()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    # node.init_sort()
    # node.init_csv_log()

    while True:
        start = dt.datetime.utcnow()

        input_img = node.read_video()  # read and resize image

        detections = detect(input_img)  # yolo inference
        node.publish_person_coordinates(detections)
        result_img = draw_results(input_img, detections)  # yolo inference
        boxes, classes, scores = node.peson_checker(detections)

        node.run_sort(boxes, scores)  # object tracking

        duration = dt.datetime.utcnow() - start
        node.fps = int(round(1000000 / duration.microseconds))

        node.duration_sec = (duration.microseconds / 1000000)
        node.runtime_sec += node.duration_sec
        node.runtime_sec = round(node.runtime_sec, 3)

        node.run_csv_log()
        node.detect_fall()

        node.publish_fps()
        node.publish_owner_exists()
        node.publish_owner_size()
        node.publish_owner_center()
        if node.owner_fall:
            node.publish_owner_fall()

        cv2.putText(result_img, f'fps: {node.fps}',
                    (25, 50), 1, 2, (0, 255, 0), 2)
        cv2.imshow("result", result_img)

        # If the `q` key was pressed, break from the loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    rclpy.shutdown()
    node.cap.release()
    cv2.destroyAllWindows()


#######################################################################################################################

if __name__ == '__main__':
    main()
