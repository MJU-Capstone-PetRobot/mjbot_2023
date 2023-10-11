#!/usr/bin/python3
import os
from Chat.voiceChat import *


# 명자 객체 형성
mj = MYOUNGJA("순자", "she")
# 먼저 말 거는 기능 실험
# mj.speak_first_ex
# 먼저 말 거는 기
mj.speak_first()
    # 대화 시작
response = "내가 사람을 쳤어"

mj.speaker("네",1,2)

emotion = mj.gpt_send_anw(response)[0]
ans_emotion = 0
emotion_strength = 1

# NULL, close, moving, wink, angry, sad, daily
if emotion == "분노":
    ans_emotion = 3
    emotion_strength = 2
elif emotion == "슬픔":
    ans_emotion = 1
    emotion_strength = 0

ans = mj.gpt_send_anw(response)[1]

mj.speaker(ans,emotion_strength,emotion)

