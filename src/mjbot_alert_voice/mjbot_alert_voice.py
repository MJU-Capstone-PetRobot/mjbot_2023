#!/usr/bin/env python
import os
from Chat.voiceChat import *
# from Alert.messageSending import *

# 화재 감지 후, 메세지 송신
# fire = fire_check()
# if fire == 2:
#     send_message(fire)

# 명자 객체 형성
mj = MYOUNGJA("순자","she")

# 대화 시작
response = mj.mic()

if response == "로봇":
    mj.speaker("네")

    while response != "":
        response = mj.mic()

        emotion = mj.gpt_send_anw[0]

        # 기쁨, 슬픔, 평범, 당황, 분노, 사랑, 위험
        if emotion == "기쁨":
            voicePublish = 11
        elif emotion == "당황":
            voicePublish = 12
        elif emotion == "분노":
            voicePublish = 13
        elif emotion == "위험":
            voicePublish = 14
        elif emotion == "슬픔":
            voicePublish = 15
        elif emotion == "사랑":
            voicePublish = 16
        elif emotion == "평범" and response == "왼손": # 왼손
            voicePublish = 1
        elif emotion == "평범" and response == "오른손": # 오른손
            voicePublish = 2
        else:
            voicePublish = 0

        os.remove("sampleWav.wav")

        ans = mj.gpt_send_anw(response)[1]

        mj.speaker(ans)

# 먼저 말 거는 기능
mj.speak_first()
