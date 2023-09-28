import cv2
import numpy as np

color_list = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
skeleton = [[16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12], [7, 13], [6, 7], [6, 8], [7, 9], [8, 10], [9, 11],
            [2, 3], [1, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7]]

CLASSES = ['person']

meshgrid = []

class_num = len(CLASSES)
headNum = 3
keypoint_num = 17

strides = [8, 16, 32]
mapSize = [[80, 80], [40, 40], [20, 20]]
nmsThresh = 0.55
objectThresh = 0.5

input_imgH = 640
input_imgW = 360


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


# def xywh2xyxy(x):
#     # Convert [x, y, w, h] to [x1, y1, x2, y2]
#     y = np.copy(x)
#     y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
#     y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
#     y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
#     y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
#     return y


class DetectBox:
    def __init__(self, classId, score, xmin, ymin, xmax, ymax, pose):
        self.classId = classId
        self.score = score
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.pose = pose


def GenerateMeshgrid():
    for index in range(headNum):
        for i in range(mapSize[index][0]):
            for j in range(mapSize[index][1]):
                meshgrid.append(j + 0.5)
                meshgrid.append(i + 0.5)


def IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
    xmin = max(xmin1, xmin2)
    ymin = max(ymin1, ymin2)
    xmax = min(xmax1, xmax2)
    ymax = min(ymax1, ymax2)

    innerWidth = xmax - xmin
    innerHeight = ymax - ymin

    innerWidth = innerWidth if innerWidth > 0 else 0
    innerHeight = innerHeight if innerHeight > 0 else 0

    innerArea = innerWidth * innerHeight

    area1 = (xmax1 - xmin1) * (ymax1 - ymin1)
    area2 = (xmax2 - xmin2) * (ymax2 - ymin2)

    total = area1 + area2 - innerArea

    return innerArea / total


def NMS(detectResult):
    predBoxs = []

    sort_detectboxs = sorted(detectResult, key=lambda x: x.score, reverse=True)

    for i in range(len(sort_detectboxs)):
        xmin1 = sort_detectboxs[i].xmin
        ymin1 = sort_detectboxs[i].ymin
        xmax1 = sort_detectboxs[i].xmax
        ymax1 = sort_detectboxs[i].ymax
        classId = sort_detectboxs[i].classId

        if sort_detectboxs[i].classId != -1:
            predBoxs.append(sort_detectboxs[i])
            for j in range(i + 1, len(sort_detectboxs), 1):
                if classId == sort_detectboxs[j].classId:
                    xmin2 = sort_detectboxs[j].xmin
                    ymin2 = sort_detectboxs[j].ymin
                    xmax2 = sort_detectboxs[j].xmax
                    ymax2 = sort_detectboxs[j].ymax
                    iou = IOU(xmin1, ymin1, xmax1, ymax1,
                              xmin2, ymin2, xmax2, ymax2)
                    if iou > nmsThresh:
                        sort_detectboxs[j].classId = -1
    return predBoxs


def postprocess(out, img_h, img_w):
    print('postprocess ... ')

    detectResult = []

    output = []
    for i in range(len(out)):
        output.append(out[i].reshape((-1)))

    scale_h = img_h / input_imgH
    scale_w = img_w / input_imgW

    gridIndex = -2

    for index in range(headNum):
        reg = output[index * 2 + 0]
        cls = output[index * 2 + 1]
        pose = output[headNum * 2 + index]

        for h in range(mapSize[index][0]):
            for w in range(mapSize[index][1]):
                gridIndex += 2

                for cl in range(class_num):
                    cls_val = sigmoid(
                        cls[cl * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w])

                    if cls_val > objectThresh:
                        x1 = (meshgrid[gridIndex + 0] - reg[0 * mapSize[index][0] *
                              mapSize[index][1] + h * mapSize[index][1] + w]) * strides[index]
                        y1 = (meshgrid[gridIndex + 1] - reg[1 * mapSize[index][0] *
                              mapSize[index][1] + h * mapSize[index][1] + w]) * strides[index]
                        x2 = (meshgrid[gridIndex + 0] + reg[2 * mapSize[index][0] *
                              mapSize[index][1] + h * mapSize[index][1] + w]) * strides[index]
                        y2 = (meshgrid[gridIndex + 1] + reg[3 * mapSize[index][0] *
                              mapSize[index][1] + h * mapSize[index][1] + w]) * strides[index]

                        xmin = x1 * scale_w
                        ymin = y1 * scale_h
                        xmax = x2 * scale_w
                        ymax = y2 * scale_h

                        xmin = xmin if xmin > 0 else 0
                        ymin = ymin if ymin > 0 else 0
                        xmax = xmax if xmax < img_w else img_w
                        ymax = ymax if ymax < img_h else img_h

                        poseResult = []
                        for kc in range(keypoint_num):
                            px = pose[(kc * 3 + 0) * mapSize[index][0] *
                                      mapSize[index][1] + h * mapSize[index][1] + w]
                            py = pose[(kc * 3 + 1) * mapSize[index][0] *
                                      mapSize[index][1] + h * mapSize[index][1] + w]
                            vs = sigmoid(
                                pose[(kc * 3 + 2) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w])

                            x = (
                                px * 2.0 + (meshgrid[gridIndex + 0] - 0.5)) * strides[index] * scale_w
                            y = (
                                py * 2.0 + (meshgrid[gridIndex + 1] - 0.5)) * strides[index] * scale_h

                            poseResult.append(vs)
                            poseResult.append(x)
                            poseResult.append(y)
                        # print(poseResult)
                        box = DetectBox(cl, cls_val, xmin, ymin,
                                        xmax, ymax, poseResult)
                        detectResult.append(box)

    # NMS
    print('detectResult:', len(detectResult))
    predBox = NMS(detectResult)

    return predBox


# def draw(image, boxes, scores, classes):
#     """Draw the boxes on the image.
#     # Argument:
#         image: original image.
#         boxes: ndarray, boxes of objects.
#         classes: ndarray, classes of objects.
#         scores: ndarray, scores of objects.
#         all_classes: all classes name.
#     """
#     p_size = [0, 0]  # w, h
#     p_center = [0, 0]  # cx, cy

#     for box, score, cl in zip(boxes, scores, classes):
#         left, top, right, bottom = box
#         print('class: {}, score: {}'.format(CLASSES[cl], score))
#         print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(
#             top, left, right, bottom))
#         top = int(top)
#         left = int(left)
#         right = int(right)
#         bottom = int(bottom)

#         p_color = (0, 255, 0)
#         else_color = (255, 0, 0)

#         if CLASSES[cl] == 'person':
#             box_color = p_color

#             p_size[0] = (int)(right - left)  # w
#             p_size[1] = (int)(bottom - top)  # h

#             p_center[0] = (right + left) // 2  # cx
#             p_center[1] = (bottom + top) // 2  # cy

#             cv2.putText(image, 'w: {} h: {}'.format(
#                 p_size[0], p_size[1]), (p_center[0], top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, p_color, 2)
#             cv2.putText(image, 'cx: {} cy: {}'.format(p_center[0], p_center[1]), (
#                 p_center[0], p_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, p_color, 2)
#         else:
#             box_color = else_color

#         cv2.rectangle(image, (left, top), (right, bottom), box_color, 2)
#         cv2.putText(image, '{0} {1:.2f}'.format(
#             CLASSES[cl], score), (left, top - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#     return p_size, p_center
