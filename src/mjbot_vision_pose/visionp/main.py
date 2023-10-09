import datetime as dt
import numpy as np
import cv2
from rknnlite.api import RKNNLite

# from vision.depth import *
# from .sort import Sort

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange

import csv
from yolo import *

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from example_interfaces.msg import Int32
from example_interfaces.msg import Int16MultiArray
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

import os
import urllib
import traceback
import time
import sys

import cv2
from math import exp

RKNN_MODEL = '/home/mju/Desktop/mjbot_2023/src/mjbot_vision/vision/yolov8pos_relu_zq.rknn'  # 절대경로


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_process")
        GenerateMeshgrid()

        self.fps = 0
        self.duration_sec = 0
        self.runtime_sec = 0

        self.owner_size = [0, 0]  # [w, h]
        self.owner_center = [0, 0]  # [cx, cy]
        self.owner_size_prev = [0, 0]  # for fall detection
        self.owner_size_diff = [0, 0]  # [w_diff, h_diff]
        self.owner_exists = False
        self.owner_fall = False

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

    def init_rknn(self):
        # Using verbose option saves lots of state
        self.rknn_lite = RKNNLite(verbose=False)

        # Load RKNN model
        self.get_logger().info('RKNN: Load RKNN model')
        ret = self.rknn_lite.load_rknn(RKNN_MODEL)
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

    def run_rknn(self, img):
        # Inference
        self.get_logger().info('RKNN: Inference')
        outputs = self.rknn_lite.inference(inputs=[img])
        self.get_logger().info('RKNN: Done')

        return outputs

    def drawing_box(self, outputs, img_h, img_w, orig_img):

        out = []
        for i in range(len(outputs)):
            out.append(outputs[i])
        predbox = postprocess(out, img_h, img_w)

        print('obj num is :', len(predbox))

        for i in range(len(predbox)):
            xmin = int(predbox[i].xmin)
            ymin = int(predbox[i].ymin)
            xmax = int(predbox[i].xmax)
            ymax = int(predbox[i].ymax)
            classId = predbox[i].classId
            score = predbox[i].score

            cv2.rectangle(orig_img, (xmin, ymin),
                          (xmax, ymax), (128, 72, 0), 2)
            ptext = (xmin, ymin)
            title = CLASSES[classId] + "%.2f" % score
            cv2.putText(orig_img, title, ptext, cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2, cv2.LINE_AA)

            pose = predbox[i].pose
            for i in range(0, keypoint_num):
                if pose[i * 3] > 0.5:
                    if i < 5:
                        color = color_list[0]
                    elif 5 <= i < 12:
                        color = color_list[1]
                    else:
                        color = color_list[2]
                    cv2.circle(
                        orig_img, (int(pose[i * 3 + 1]), int(pose[i * 3 + 2])), 2, color, 5)

            for i, sk in enumerate(skeleton):
                if pose[(sk[0] - 1) * 3] > 0.5:
                    pos1 = (int(pose[(sk[0] - 1) * 3 + 1]),
                            int(pose[(sk[0] - 1) * 3 + 2]))
                    pos2 = (int(pose[(sk[1] - 1) * 3 + 1]),
                            int(pose[(sk[1] - 1) * 3 + 2]))
                    if (sk[0] - 1) < 5:
                        color = color_list[0]
                    elif 5 <= sk[0] - 1 < 12:
                        color = color_list[1]
                    else:
                        color = color_list[2]
                    cv2.line(orig_img, pos1, pos2, color,
                             thickness=2, lineType=cv2.LINE_AA)

        return orig_img
        # # Post process
        # input0_data = outputs[0]
        # input1_data = outputs[1]
        # input2_data = outputs[2]

        # input0_data = input0_data.reshape([3, -1]+list(input0_data.shape[-2:]))
        # input1_data = input1_data.reshape([3, -1]+list(input1_data.shape[-2:]))
        # input2_data = input2_data.reshape([3, -1]+list(input2_data.shape[-2:]))

        # input_data = list()
        # input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        # input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        # input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        # # Yolov5 post process
        # self.boxes, self.classes, self.scores = vision.yolo.yolov5_post_process(input_data)

        # # Draw detection box
        # if self.boxes is not None:
        #     self.owner_size, self.owner_center = vision.yolo.draw(self.img, self.boxes, self.scores, self.classes)
        #     self.get_logger().info("Owner info (w: {}, h: {}, cx: {}, cy: {})".format(self.owner_size[0], self.owner_size[1], self.owner_center[0], self.owner_center[1]))
        # else:
        #     self.owner_size = [0, 0]
        #     self.owner_center = [0, 0]

    def init_sort(self):
        self.sort = Sort(max_age=2, min_hits=3, iou_threshold=0.3, )

    def run_sort(self):
        if self.boxes is not None:
            self.scores = self.scores.reshape((-1, 1))
            dets = np.concatenate((self.boxes, self.scores), 1)
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


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    node.init_video()
    node.init_rknn()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    # node.init_sort()
    # node.init_csv_log()

    while True:
        start = dt.datetime.utcnow()

        input_img = node.read_video()  # read and resize image
        output_img = node.run_rknn(input_img)  # yolo inference
        result_img = node.drawing_box(output_img, 640, 360, input_img)
        # node.run_sort() # object tracking

        duration = dt.datetime.utcnow() - start
        node.fps = int(round(1000000 / duration.microseconds))

        node.duration_sec = (duration.microseconds / 1000000)
        node.runtime_sec += node.duration_sec
        node.runtime_sec = round(node.runtime_sec, 3)

        # node.run_csv_log()
        # node.detect_fall()

        node.publish_fps()
        node.publish_owner_exists()
        node.publish_owner_size()
        node.publish_owner_center()
        # if node.owner_fall:
        #     node.publish_owner_fall()
        # publish emotion
        # publish depth

        cv2.putText(result_img, f'fps: {node.fps}',
                    (25, 50), 1, 2, (0, 255, 0), 2)
        cv2.imshow("result", result_img)

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
