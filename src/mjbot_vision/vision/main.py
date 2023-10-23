import os

import datetime as dt
import numpy as np
import cv2
from rknnlite.api import RKNNLite

from vision.depth import *
from vision.sort import Sort
from vision import yolov5
from vision import yolov7

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
from example_interfaces.msg import Bool
from example_interfaces.msg import Int32
from example_interfaces.msg import Int16MultiArray

# RKNN_MODEL = 'yolov5s-640-640-rk3588.rknn'
RKNN_MODEL = 'yolov7-tiny_tk2_RK3588_i8.rknn'


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_process")

        self.fps = 0
        self.duration_sec = 0
        self.runtime_sec = 0

        self.owner_x : int = 0
        self.owner_y : int = 0
        self.owner_z : int = 0

        self.owner_exists = False
        self.owner_fall = False

        self.owner_w : int = 0
        self.owner_h : int = 0
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

        self.publisher_fps_ = self.create_publisher(Int32, "fps", 10)
        self.publisher_owner_exists_ = self.create_publisher(
            Bool, "owner_exists", 10)
        self.publisher_owner_center_ = self.create_publisher(
            Int16MultiArray, "owner_center", 10)
        self.publisher_owner_size_ = self.create_publisher(
            Int16MultiArray, "owner_size", 10)
        self.publisher_owner_fall_ = self.create_publisher(
            Bool, "owner_fall", 10)
        self.get_logger().info("Node has been started")

    def publish_fps(self):
        msg = Int32()
        msg.data = self.fps
        self.publisher_fps_.publish(msg)
        self.get_logger().info("PUB: /fps: {}".format(msg.data))

    def publish_owner_exists(self):
        msg = Bool()
        msg.data = self.owner_exists
        self.publisher_owner_exists_.publish(msg)
        self.get_logger().info("PUB: /owner_fall: {}".format(msg.data))

    def publish_owner_size(self):
        msg = Int16MultiArray()
        msg.data = [self.owner_w, self.owner_h]
        self.publisher_owner_size_.publish(msg)
        self.get_logger().info("PUB: /owner_size: {}".format(msg.data))

    def publish_owner_center(self):
        msg = Int16MultiArray()

        msg.data = [self.owner_x, self.owner_y, self.owner_z]
        self.publisher_owner_center_.publish(msg)
        self.get_logger().info("PUB: /owner_center: {}".format(msg.data))

    def publish_owner_fall(self):
        msg = Bool()
        msg.data = self.owner_fall
        self.publisher_owner_fall_.publish(msg)
        self.get_logger().info("PUB: /owner_fall: {}".format(msg.data))

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

    def run_rknn(self):
        # Inference
        self.get_logger().info('RKNN: Inference')
        outputs = self.rknn_lite.inference(inputs=[self.img])
        self.get_logger().info('RKNN: Done')

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
        if self.p_boxes is not None:
            self.p_scores = self.p_scores.reshape((-1, 1))
            dets = np.concatenate((self.p_boxes, self.p_scores), 1) 
            tracker = self.sort.update(dets)

            first_id_index = len(tracker) - 1 # sort output is id descending order

            for idx, trk in enumerate(tracker):
                trk = trk.astype(int)

                if idx == first_id_index: # Get first-id coordinate
                    cv2.rectangle(self.img, (trk[0], trk[1]), (trk[2], trk[3]), (0, 255, 0), 2)
                    cv2.putText(self.img,  "ID:"+str(trk[4]), (trk[0], trk[1] + 12), 1, 1, (255, 255, 255), 2)

                    self.owner_exists = True

                    self.owner_x = (int)((trk[0] + trk[2]) // 2)
                    self.owner_y = (int)((trk[1] + trk[3]) // 2)

                    self.owner_w = (int)(trk[2] - trk[0])
                    self.owner_h = (int)(trk[3] - trk[1])

                    # Get depth distance
                    depth_x = self.owner_x
                    depth_y = self.owner_y - 80
                    self.owner_z = (int)(self.dep[depth_y, depth_x])  # ordered y, x

                    cv2.putText(self.img, "X {}".format(self.owner_x), (570, 30), 1, 1, (0, 255, 0), 2)
                    cv2.putText(self.img, "Y {}".format(self.owner_y), (570, 50), 1, 1, (0, 255, 0), 2)
                    cv2.putText(self.img, "Z {}".format(self.owner_z), (570, 70), 1, 1, (0, 255, 0), 2)

                    cv2.putText(self.img, "W {}".format(self.owner_w), (500, 30), 1, 1, (0, 255, 0), 2)
                    cv2.putText(self.img, "H {}".format(self.owner_h), (500, 50), 1, 1, (0, 255, 0), 2)
                else:
                    cv2.rectangle(self.img, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 2)
                    cv2.putText(self.img,  "ID:"+str(trk[4]), (trk[0], trk[1] + 12), 1, 1, (255, 255, 255), 2)
            
    def detect_fall(self):
        if self.owner_exists:
            self.owner_w_diff = (self.owner_w - self.owner_w_prev) / self.duration_sec
            self.owner_h_diff = (self.owner_h - self.owner_h_prev) / self.duration_sec 

            if self.owner_h_diff <= -500:
                self.owner_fall = True
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
        with open('/home/drcl/Desktop/mjbot_2023/src/mjbot_vision/vision/log.csv', 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()

    def run_csv_log(self):
        with open('/home/drcl/Desktop/mjbot_2023/src/mjbot_vision/vision/log.csv', 'a') as csv_file:
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


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    # node.init_video("webcam")
    node.init_depth()
    node.init_rknn()
    node.init_sort()
    # node.init_csv_log()

    executor = MultiThreadedExecutor() # 노드 내부의 콜백 함수를 스레드로 잘라서 실행
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin) # 메인 함수에서 자식 스레드를 만들어서 Ros 코드를 돌림
    executor_thread.start()

    while True:
        start = dt.datetime.utcnow()

        # node.read_video("webcam") # read and resize image
        node.read_depth(False)
        node.run_rknn() # yolo inference
        node.run_sort() # object tracking

        duration = dt.datetime.utcnow() - start
        node.fps = int(round(1000000 / duration.microseconds))

        node.duration_sec = (duration.microseconds / 1000000)
        node.runtime_sec += node.duration_sec
        node.runtime_sec = round(node.runtime_sec, 3)

        # node.run_csv_log()
        node.detect_fall()  
        node.publish_fps()
        node.publish_owner_exists()

        if node.owner_exists:
            node.publish_owner_size()
            node.publish_owner_center()
        if node.owner_fall:
            node.publish_owner_fall()

        cv2.putText(node.img, f'fps: {node.fps}', (25, 50), 1, 2, (0, 255, 0), 2)
        cv2.imshow("result", node.img)

        # If the `q` key was pressed, break from the loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    rclpy.shutdown()
    node.rknn_lite.release()
    node.cap.release()
    cv2.destroyAllWindows()


#######################################################################################################################

if __name__ == '__main__':
    main()
