#!/usr/bin/env python3

import os
import cv2
import sys
import argparse

import rclpy
from utils.coco_utils import COCO_test_helper
import numpy as np
from PIL import Image, ImageDraw
import numpy as np
import cv2
import random

from rknnlite.api import RKNNLite


vision_package_directory = get_package_share_directory("mjbot_vision2")

model = RKNNLite()
model.load_rknn(os.path.join(vision_package_directory, 'model', 'yolov7.rknn'))
model.init_runtime()

ANCHORS = [12.0, 16.0, 19.0, 36.0, 40.0, 28.0, 36.0, 75.0, 76.0,
          55.0, 72.0, 146.0, 142.0, 110.0, 192.0, 243.0, 459.0, 401.0]


IMG_SIZE = (640, 640)
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
CLASSES = ("person", "bicycle", "car", "motorbike ", "aeroplane ", "bus ", "train", "truck ", "boat", "traffic light",
           "fire hydrant", "stop sign ", "parking meter", "bench", "bird", "cat", "dog ", "horse ", "sheep", "cow", "elephant",
           "bear", "zebra ", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
           "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife ",
           "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza ", "donut", "cake", "chair", "sofa",
           "pottedplant", "bed", "diningtable", "toilet ", "tvmonitor", "laptop	", "mouse	", "remote ", "keyboard ", "cell phone", "microwave ",
           "oven ", "toaster", "sink", "refrigerator ", "book", "clock", "vase", "scissors ", "teddy bear ", "hair drier", "toothbrush ")


colors = tuple(tuple(random.randint(0, 255) for _ in range(3)) for _ in range(len(CLASSES)))

cutout_mask = None
highlight_pts = None
highlight_mask = None
co_helper = COCO_test_helper(enable_letter_box=True)
def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    if class_num==1:
        _class_pos = np.where(box_confidences >= OBJ_THRESH)
        scores = (box_confidences)[_class_pos]
    else:
        _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
        scores = (class_max_score* box_confidences)[_class_pos]


    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep



def box_process(position, anchors):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0] //
                      grid_w]).reshape(1, 2, 1, 1)

    col = col.repeat(len(anchors), axis=0)
    row = row.repeat(len(anchors), axis=0)
    anchors = np.array(anchors)
    anchors = anchors.reshape(*anchors.shape, 1, 1)

    box_xy = position[:, :2, :, :]*2 - 0.5
    box_wh = pow(position[:, 2:4, :, :]*2, 2) * anchors

    box_xy += grid
    box_xy *= stride
    box = np.concatenate((box_xy, box_wh), axis=1)

    # Convert [c_x, c_y, w, h] to [x1, y1, x2, y2]
    xyxy = np.copy(box)
    xyxy[:, 0, :, :] = box[:, 0, :, :] - box[:, 2, :, :] / 2  # top left x
    xyxy[:, 1, :, :] = box[:, 1, :, :] - box[:, 3, :, :] / 2  # top left y
    xyxy[:, 2, :, :] = box[:, 0, :, :] + \
        box[:, 2, :, :] / 2  # bottom right x
    xyxy[:, 3, :, :] = box[:, 1, :, :] + \
        box[:, 3, :, :] / 2  # bottom right y

    return xyxy


def post_process(input_data, anchors):
    boxes, scores, classes_conf = [], [], []

    input_data = [_in.reshape(
        [len(anchors[0]), -1]+list(_in.shape[-2:])) for _in in input_data]
    for i in range(len(input_data)):
        boxes.append(box_process(
            input_data[i][:, :4, :, :], anchors[i]))
        scores.append(input_data[i][:, 4:5, :, :])
        classes_conf.append(input_data[i][:, 5:, :, :])

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # filter according to threshold
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []

    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores



# Pillow를 사용하여 이미지에 경계 상자 및 레이블을 그리는 함수입니다.
# 이미지를 수정하지 않고 새 이미지를 반환합니다.
def draw_results(image, results):
    image = Image.fromarray(image)
    draw = ImageDraw.Draw(image)
    for box, cl, score in results:
        text = CLASSES[cl]  # 클래스 이름을 가져옵니다.
        draw.rectangle(box, outline=colors[cl], width=2)  # 경계 상자 그리기
        label_box = draw.textbbox((0,0), text)  # 레이블의 위치 계산
        y = box[1] - label_box[3]
        if y < 0:
            y = box[3] + 1
        draw.text((box[0], y), text, colors[cl])  # 레이블 그리기
    return np.asarray(image)


# 원본 이미지를 수정하지 않고 마스크를 적용한 이미지를 반환합니다.
def make_mask(image):
    return cv2.add(image, 0, mask=cutout_mask) if cutout_mask is not None else image

# 이미지 위에 마스크를 그리는 함수입니다.
def draw_mask(image):
    if highlight_mask is not None:
        cv2.addWeighted(image, 1, highlight_mask, 0.2, 0, image)
        cv2.polylines(image, highlight_pts, True, (255, 255, 0), 1)

# 이미지에서 객체를 검출하는 함수입니다.
def detect(image):
    img, ratio, (dw, dh) = co_helper.letter_box(im=image, new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=(0,0,0), info_need=True)

    outputs = model.inference([cv2.cvtColor(make_mask(img), cv2.COLOR_BGR2RGB)])
    boxes, classes, scores = post_process(outputs, ANCHORS)

    if boxes is not None:
        for i in range(len(boxes)):
            bbox = boxes[i]
            bbox[0] -= dw
            bbox[1] -= dh
            bbox[2] -= dw
            bbox[3] -= dh
            boxes[i] = [value/ratio for value in bbox]
        
    return [(
        tuple(map(int, item[0])),  # box
        item[1],  # name
        item[2]  # score
     ) for item in zip(boxes, classes, scores)] if boxes is not None else []

# 객체 검출 임계값을 설정하는 함수입니다.
def set_obj_thresh(thresh):
    global OBJ_THRESH
    OBJ_THRESH = thresh

