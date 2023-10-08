import cv2
import time
from enum import Enum

class ObjectTracer:
    TRACE_TIME_THRESHHOLD = 1.5  # 개체 상태를 확인하는 데 필요한 시간(초)
    TRACE_ENTER_PERCENT_THRESHHOLD = 0.8  # 개체가 들어온 것으로 간주되는 검출 비율(없다가 있게 됨)
    TRACE_LEAVE_PERCENT_THRESHHOLD = 0.  # 개체가 나간 것으로 간주되는 검출 비율(있다가 없어짐)
    TRACE_MIN_SIM = 0.5  # 최소 유사도, 이 값보다 높으면 이미지에 있던 개체로 간주
    DISABLE_DRAW_ALERT_BOX = False  # 경고 상자 그리기를 비활성화할지 여부

    class Action(Enum):
        NONE = 0
        ENTER = 1
        LEAVE = 2
        FAKE = 3  # 이전에 존재하지 않았고 검출 후에도 존재하지 않은 경우 트리거됨, 해당 개체를 더 이상 추적하지 않음

    def __init__(self, label, box, image):
        self.label = label
        self.box = box
        self.enter_pos = self.get_pos()
        self.create_time = time.perf_counter()
        self.image = image
        self.exists = False
        self.linkage_alerted = False  # 이미 연합 경고에 참여한 경우(여러 대상에 대한 연합 경고의 경우 여러 번 알림이 발생함)
        self._last_trace_time = self.create_time  # 마지막 검출 시간
        self._ex_trace_count = 1  # 존재하는 프레임 수 기록
        self._total_trace_count = 1  # 총 검출된 프레임 수 기록
    
    def sim(self, box):
        xmin1, ymin1, xmax1, ymax1 = self.box
        xmin2, ymin2, xmax2, ymax2 = box
        w1, h1, w2, h2 = xmax1 - xmin1, ymax1 - ymin1, xmax2 - xmin2, ymax2 - ymin2

        inter = max(0, min(xmax1, xmax2) - max(xmin1, xmin2)) * max(0, min(ymax1, ymax2) - max(ymin1, ymin2))
        union = w1 * h1 + w2 * h2 - inter
        if inter <= 0 or union <= 0:
            iou = 0
        else:
            iou = inter / union

        xmin = min(xmin1, xmin2)
        ymin = min(ymin1, ymin2)
        xmax = max(xmax1, xmax2)
        ymax = max(ymax1, ymax2)
        w, h = xmax - xmin, ymax - ymin

        # 중심점 거리 유사도(0~1)
        try:
            dis_sim = 1 - (((xmin1 + xmax1) / 2 - (xmin2 + xmax2) / 2) ** 2 + ((ymin1 + ymax1) / 2 - (ymin2 + ymax2) / 2) ** 2) / (w ** 2 + h ** 2)
        except ZeroDivisionError:
            dis_sim = 1

        # 형상 유사도
        try:
            shape_sim = 1 - ((w1 - w2) ** 2 / w ** 2 + (h1 - h2) ** 2 / h ** 2) / 2
        except ZeroDivisionError:
            shape_sim = 1
        return iou * 0.4 + dis_sim * 0.2 + shape_sim * 0.4
    
    def update(self, time, box=None, image=None):
        # box가 None인 경우 해당 개체가 존재하지 않음을 나타냄, 빈 업데이트 실행
        if box is not None:
            self._ex_trace_count += 1
            self.box = box
            self.image = image
        self._total_trace_count += 1

        if time - self._last_trace_time > self.TRACE_TIME_THRESHHOLD:  # 한 번의 추적 검출 후 현재 상태를 기반으로 작업 업데이트
            self._last_trace_time = time
            # 검출 비율 계산
            trace_percent = self._ex_trace_count / self._total_trace_count if self._total_trace_count else 0
            # 카운터를 리셋
            self._ex_trace_count = self._total_trace_count = 0

            if not self.exists:
                if trace_percent >= self.TRACE_ENTER_PERCENT_THRESHHOLD:
                    # 들어올 때, 들어올 검출 비율 임계값을 만족해야 함
                    self.exists = True
                    return self.Action.ENTER
                else:
                    return self.Action.FAKE
            elif trace_percent <= self.TRACE_LEAVE_PERCENT_THRESHHOLD:  # 여기로 오면 이미 존재함
                # 나갈 때, 나갈 검출 비율 임계값보다 작아야 함, 검출 비율의 판단은 약간 널널하게 해야 함, 단 한 번만 검출되어도 개체가 존재함을 증명할 수 있음
                return self.Action.LEAVE
        return self.Action.NONE
    
    # 이 개체에 대한 이미지를 가져오며, 존재하거나 들어온 경우에만 이미지를 설정함, 나가거나 가짜인 경우 이미지를 설정하지 않음
    def make_shotcut(self, image=None):
        if image is None:
            image = self.image.copy()
        if not self.DISABLE_DRAW_ALERT_BOX:
            cv2.rectangle(image, self.box[:2], self.box[2:4], (247, 200, 125), 2)
        return image
    
    def get_pos(self):
        return ((self.box[0] + self.box[2]) / 2, (self.box[1] + self.box[3]) / 2)
