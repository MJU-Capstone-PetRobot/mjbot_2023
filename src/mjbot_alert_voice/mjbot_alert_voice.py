import os
from Chat.voiceChat import *
from Alert.messageSending import *

# 화재 감지 후, 메세지 송신
fire = fire_check()
if fire == 2:
    send_message(fire)

# 명자 객체 형성
mj = MYOUNGJA()

# 대화 시작
response = mj.mic()

if response == "로봇":
    mj.speaker("네")

    while response != "":
        response = mj.mic()

        if mj.findNegative(response) > 0: # 부정적 발화
            voicePublish = 3
        elif response == "왼손": # 왼손
            voicePublish = 1
        elif response == "오른손": # 오른손
            voicePublish = 2
        else:
            voicePublish = 0

        os.remove("sampleWav.wav")

        ans = mj.gpt_send_anw(response)

        mj.speaker(ans)

# 먼저 말 거는 기능
mj.speak_first()
