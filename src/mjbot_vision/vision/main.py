import os

import datetime as dt
import numpy as np
import cv2
from rknnlite.api import RKNNLite
from hide_warnings import hide_warnings
from vision.depth import *
from vision.sort import Sort
from vision import yolov5
from vision import yolov7
import time

import csv
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int16MultiArray

# RKNN_MODEL = 'yolov5s-640-640-rk3588.rknn'
RKNN_MODEL = 'yolov7-tiny_tk2_RK3588_i8.rknn'


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_process")

        self.fps = 0
        self.duration_sec = 0
        self.runtime_sec = 0

        self.owner_x: int = 0
        self.owner_y: int = 0
        self.owner_z: int = 0

        self.owner_exists = False
        self.owner_fall = False

        self.owner_w: int = 0
        self.owner_h: int = 0
        self.owner_w_prev = 0
        self.owner_h_prev = 0
        self.owner_w_diff = 0
        self.owner_h_diff = 0

        self.p_boxes = np.array([])
        self.p_scores = np.array([])

        self.fieldnames = ["duration", "runtime", "fps",
                           "w", "h",
                           "w_diff", "h_diff",
                           "exists", "fall"]

        self.publisher_owner_size_ = self.create_publisher(
            Int16MultiArray, "owner_size", 10)
        self.publisher_owner_xyz_ = self.create_publisher(
            Int16MultiArray, "owner_xyz", 10)
        self.publisher_owner_fall_ = self.create_publisher(
            Bool, "owner_fall", 10)

        self.timer_ = self.create_timer(0.01, self.publish_owner_size_and_xyz)
        self.process_timer = self.create_timer(1.0/15, self.process_callback)

        self.get_logger().info("Vision Node has been started")

    def publish_owner_size_and_xyz(self):
        msg = Int16MultiArray()

        # Clamp values to the int16 range and ensure they are integers
        clamped_w = max(-32768, min(32767, int(self.owner_w)))
        clamped_h = max(-32768, min(32767, int(self.owner_h)))
        msg.data = [clamped_w, clamped_h]
        self.publisher_owner_size_.publish(msg)

        msg = Int16MultiArray()
        clamped_x = max(-32768, min(32767, int(self.owner_x)))
        clamped_y = max(-32768, min(32767, int(self.owner_y)))
        clamped_z = max(-32768, min(32767, int(self.owner_z)))
        msg.data = [clamped_x, clamped_y, clamped_z]
        self.publisher_owner_xyz_.publish(msg)


    def publish_owner_fall(self):  # event
        msg = Bool()
        msg.data = self.owner_fall
        self.publisher_owner_fall_.publish(msg)
        # self.get_logger().info("PUB: /owner_fall: {}".format(msg.data))

    def init_video(self, source):
        if source == "webcam":
            self.cap = cv2.VideoCapture(0)
        elif source == "video":
            self.cap = cv2.VideoCapture(
                '/home/drcl/ros2_ws/src/vision/vision/video/fall-01.mp4')

        if not self.cap.isOpened():
            self.get_logger().info("VIDEO: Cannot open video")
        else:
            self.get_logger().info("VIDEO: Open video")

    def init_depth(self):
        self.dc = DepthCamera()

    def read_video(self, source):
        if source == "webcam":
            ret, src = self.cap.read()
            self.img = cv2.resize(src, dsize=(640, 360),
                                  interpolation=cv2.INTER_LINEAR)
            self.img = cv2.copyMakeBorder(
                self.img, 140, 140, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))

        elif source == "video":
            ret, src = self.cap.read()
            self.img = cv2.resize(src, dsize=(0, 0), fx=2,
                                  fy=2, interpolation=cv2.INTER_LINEAR)
            self.img = cv2.copyMakeBorder(
                self.img, 80, 80, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))

    def read_depth(self, aligned):
        if aligned:
            ret, self.dep, src = self.dc.get_frame_aligned()
        else:
            ret, self.dep, src = self.dc.get_frame()

        self.img = cv2.copyMakeBorder(
            src, 80, 80, 0, 0, cv2.BORDER_CONSTANT, (0, 0, 0))

    def init_rknn(self):
        current_dir = os.getcwd()
        model_dir = current_dir + '/src/mjbot_vision/vision/' + RKNN_MODEL
        print(model_dir)

        # Using verbose option saves lots of state
        self.rknn_lite = RKNNLite(verbose=False)

        # Load RKNN model
        self.get_logger().info('RKNN: Load RKNN model')
        ret = self.rknn_lite.load_rknn(model_dir)
        if ret != 0:
            self.get_logger().info('RKNN: Fail to load RKNN model')
            exit(ret)
        self.get_logger().info('RKNN: done')

        # Init runtime environment
        self.get_logger().info('RKNN: Init runtime environment')
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().info('RKNN: Fail to init runtime environment')
            exit(ret)
        self.get_logger().info('RKNN: Done')
    @hide_warnings
    def run_rknn(self):
        # Inference
        # self.get_logger().info('RKNN: Inference')
       
        outputs = self.rknn_lite.inference(inputs=[self.img])
        # self.get_logger().info('RKNN: Done')

        # yolov7 post process
        self.boxes, self.classes, self.scores = yolov7.post_process(outputs)
        # self.img = yolov7.draw(self.img, boxes, scores, classes)

        # yolov5 post process
        # self.boxes, self.classes, self.scores = yolov5.post_process(outputs)

        # extract person
        self.p_boxes = np.array([])
        self.p_scores = np.array([])

        if self.boxes is not None:
            for box, score, cl in zip(self.boxes, self.scores, self.classes):
                if yolov5.CLASSES[cl] == 'person':
                    self.p_boxes = np.append(self.p_boxes, box)
                    self.p_scores = np.append(self.p_scores, score)

    def init_sort(self):
        self.sort = Sort(max_age=2, min_hits=3, iou_threshold=0.3, )

    def run_sort(self):
        self.p_boxes = self.p_boxes.reshape((-1, 4))
        if self.p_boxes.any():
            self.p_scores = self.p_scores.reshape((-1, 1))
            dets = np.concatenate((self.p_boxes, self.p_scores), 1)
            tracker = self.sort.update(dets)

            # sort output is id descending order
            first_id_index = len(tracker) - 1

            for idx, trk in enumerate(tracker):
                trk = trk.astype(int)

                if idx == first_id_index:  # Get first-id coordinate
                    cv2.rectangle(
                        self.img, (trk[0], trk[1]), (trk[2], trk[3]), (0, 255, 0), 2)
                    cv2.putText(
                        self.img,  "ID:"+str(trk[4]), (trk[0], trk[1] + 12), 1, 1, (255, 255, 255), 2)

                    center_x = (int)((trk[0] + trk[2]) // 2)
                    center_y = (int)((trk[1] + trk[3]) // 2)
                    

                    points = [
                        (center_x, center_y),
                        (max(center_x - 30, 0), center_y),
                        (min(center_x + 30, 640 - 1), center_y),
                        (center_x, max(center_y - 30, 0)),
                        (center_x, min(center_y + 30, 360 - 1))
                    ]

                    # Get depth values and find the minimum
                    depths = sorted([self.dep[y, x] for x, y in points])

                    # Find the smallest non-zero depth
                    self.owner_z = next((d for d in depths if d != 0), 0)

               

                    # Update owner_x, owner_y, owner_w, owner_h
                    self.owner_x = center_x
                    self.owner_y = center_y
                    self.owner_w = (int)(trk[2] - trk[0])
                    self.owner_h = (int)(trk[3] - trk[1])

                    self.owner_exists = True

                    # cv2.putText(self.img, "X {}".format(
                    #     self.owner_x), (570, 30), 1, 1, (0, 255, 0), 2)
                    # cv2.putText(self.img, "Y {}".format(
                    #     self.owner_y), (570, 50), 1, 1, (0, 255, 0), 2)
                    # cv2.putText(self.img, "Z {}".format(
                    #     self.owner_z), (570, 70), 1, 1, (0, 255, 0), 2)

                    # cv2.putText(self.img, "W {}".format(
                    #     self.owner_w), (500, 30), 1, 1, (0, 255, 0), 2)
                    # cv2.putText(self.img, "H {}".format(
                    #     self.owner_h), (500, 50), 1, 1, (0, 255, 0), 2)
                else:
                    cv2.rectangle(
                        self.img, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 2)
                    cv2.putText(
                        self.img,  "ID:"+str(trk[4]), (trk[0], trk[1] + 12), 1, 1, (255, 255, 255), 2)
        else:
            self.owner_exists = False

            self.owner_x = 0
            self.owner_y = 0
            self.owner_z = 0

    def detect_fall(self):
        if self.owner_exists:
            self.owner_w_diff = (
                self.owner_w - self.owner_w_prev) / self.duration_sec
            self.owner_h_diff = (
                self.owner_h - self.owner_h_prev) / self.duration_sec

            if -700 <= self.owner_h_diff <= -600:
                self.owner_fall = False
                # time.sleep(0.5)
                ### 바꿀예정 너무 튐
            else:
                self.owner_fall = False

            self.owner_w_prev = self.owner_w
            self.owner_h_prev = self.owner_h
        # else:
            # self.owner_w_diff = 0
            # self.owner_h_diff = 0
            # self.owner_w_prev = 0
            # self.owner_h_prev = 0

    def init_csv_log(self):
        current_dir = os.getcwd()
        log_dir = current_dir + '/src/mjbot_vision/vision/log.csv'

        with open(log_dir, 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()

    def run_csv_log(self):
        current_dir = os.getcwd()
        log_dir = current_dir + '/src/mjbot_vision/vision/log.csv'

        with open(log_dir, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)

            info = {
                "duration": self.duration_sec,
                "runtime": self.runtime_sec,
                "fps": self.fps,

                "w": self.owner_w,
                "h": self.owner_h,

                "w_diff": self.owner_w_diff,
                "h_diff": self.owner_h_diff,

                "exists": self.owner_exists,
                "fall": self.owner_fall
            }

            csv_writer.writerow(info)

    def process_callback(self):
        start = dt.datetime.utcnow()

        # node.read_video("webcam") # read and resize image
        self.read_depth(False)
        self.run_rknn()  # object detecting
        self.run_sort()  # object tracking

        duration = dt.datetime.utcnow() - start
        self.fps = int(round(1000000 / duration.microseconds))
        self.duration_sec = (duration.microseconds / 1000000)
        self.runtime_sec += self.duration_sec
        self.runtime_sec = round(self.runtime_sec, 3)

        # self.run_csv_log()
        # self.detect_fall()

        if self.owner_fall:
            self.publish_owner_fall()

        # # If the `q` key was pressed, exit the process
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord("q"):
        #     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    # node.init_video("webcam")
    node.init_depth()
    node.init_rknn()
    node.init_sort()
    # node.init_csv_log()

    rclpy.spin(node)  # This will keep your node running and process callbacks

    rclpy.shutdown()
    node.rknn_lite.release()
    node.cap.release()
    cv2.destroyAllWindows()


#######################################################################################################################
if __name__ == '__main__':
    main()
