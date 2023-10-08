import time

from yolo import CLASSES, set_obj_thresh
from .object_tracer import ObjectTracer
from .triggers import *

class Alerter:

    def __init__(self, *triggers):
        # 두 가지 종류의 트리거를 구별하여 다른 시기에 트리거할 수 있도록 함
        self._obj_enter_triggers = []
        self._obj_leave_triggers = []

        for trigger in triggers:
            if trigger.is_obj_enter_trigger():
                self._obj_enter_triggers.append(trigger)
            if trigger.is_obj_leave_trigger():
                self._obj_leave_triggers.append(trigger)

        # 이러한 세트의 합집합을 사용하여 추적해야 하는 개체 집합을 계산함
        self._tracing_obj_dict = set().union(*map(lambda trigger: trigger.alert_objs, self._obj_enter_triggers + self._obj_leave_triggers))
        if not self._tracing_obj_dict:
            self._tracing_obj_dict = None
        else:
            # 이를 {클래스1: 빈 세트, 클래스2: 빈 세트...} 형식으로 처리함
            self._tracing_obj_dict = {obj: set() for obj in self._tracing_obj_dict}
        
    def update(self, image, results):
        if not self._tracing_obj_dict:
            return
        
        current_time = time.perf_counter()

        # 추적 중인 모든 객체를 가져와 나중에 누가 나타나지 않았는지 확인하기 쉽게 함
        notfound_objs = set().union(*self._tracing_obj_dict.values())
        obj_and_action = []
        for box, label_idx, _ in results:
            # 해당 클래스를 찾음
            label = CLASSES[label_idx]

            obj_set = self._tracing_obj_dict.get(label)
            if obj_set is None:
                continue

            max_sim = 0
            max_sim_obj = None
            for obj in obj_set:
                # 이 객체가 이미 신청되었으면 건너뜀
                if obj not in notfound_objs:
                    continue
                sim = obj.sim(box)
                # 가장 큰 위치 유사성 개체를 찾으며 최소 위치 유사성보다 커야 함
                if sim > max_sim and sim > ObjectTracer.TRACE_MIN_SIM:
                    max_sim = sim
                    max_sim_obj = obj
            if max_sim_obj is not None:
                # 이전에 추적한 개체를 찾을 수 있음
                notfound_objs.discard(max_sim_obj)
                action = max_sim_obj.update(current_time, box, image)
                if action != ObjectTracer.Action.NONE:
                    obj_and_action.append((max_sim_obj, action))
            else:
                # 그렇지 않으면 새로운 개체 추적기를 생성함
                obj_set.add(ObjectTracer(label, box, image))
        
        for obj in notfound_objs:
            action = obj.update(current_time)
            if action != ObjectTracer.Action.NONE:
                obj_and_action.append((obj, action))
        
        for obj, action in obj_and_action:
            # 진입 또는 나가는 개체인지 여부를 확인함
            if action == ObjectTracer.Action.ENTER:
                for trigger in self._obj_enter_triggers:
                    if obj.label in trigger.alert_objs:
                        trigger.enter(obj, self._tracing_obj_dict)
            elif action == ObjectTracer.Action.LEAVE:
                for trigger in self._obj_leave_triggers:
                    if obj.label in trigger.alert_objs:
                        trigger.leave(obj, self._tracing_obj_dict)
            
            # 개체를 계속 추적해야 하는지 여부를 확인함
            if action == ObjectTracer.Action.LEAVE or action == ObjectTracer.Action.FAKE:  # FAKE: 이 객체는 떨림으로 인해 생성되었으며 가짜임, 더 이상 추적하지 않음
                self._tracing_obj_dict[obj.label].discard(obj)

def linear(min, max, percent):
    return min + (max - min) * percent

def alert_thread_wrapper(
        detect_queue, 
        alert_collect_url, 
        disable_draw_alert_box,
        enter_alert_objs, 
        leave_alert_objs, 
        linkage_alert_objs, 
        pass_alert_objs, 
        pass_alert_directions, 
        gather_alert_objs,
        gather_alert_threshhold,
        alert_sensitivity
    ):
    
    set_alert_collect_url(alert_collect_url)

    alerter = Alerter(
        EnterTrigger(enter_alert_objs),
        LeaveTrigger(leave_alert_objs),

    )

    alert_sensitivity = 1 - alert_sensitivity
    ObjectTracer.TRACE_TIME_THRESHHOLD = linear(0.5, 2, alert_sensitivity)
    ObjectTracer.TRACE_ENTER_PERCENT_THRESHHOLD = linear(0.4, 0.9, alert_sensitivity)
    ObjectTracer.DISABLE_DRAW_ALERT_BOX = disable_draw_alert_box
    set_obj_thresh(linear(0.25, 0.8, alert_sensitivity)) 
    
    while True:
        # results가 대략 다음과 같을 것이다: [(<int x1>, <int y1>, <int x2>, <int y2>), <int 클래스 인덱스>, <float 신뢰도>), ...]
        alerter.update(*detect_queue.get())  # frame은 opencv 이미지이며 경고 시 base64 이미지가 필요하므로 이미지도 전달됨
