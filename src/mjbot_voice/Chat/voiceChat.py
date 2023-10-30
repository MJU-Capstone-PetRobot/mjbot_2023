import time

import openai

## ChatGPT
openai.api_key = "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"

# NAVER CLOVA
client_id_g = "5ezz7ibsqa"
client_secret_g = "L5sJdJ281leLtB1pNXap5sFygAsTtC1jIysck4gW"


class MYOUNGJA():
    import json
    memory_size = 100

    with open('./user_value.json', 'r') as f:
        data = json.load(f)
        nameValue = data["user_name"]
        manWomanValue = data["user_value"]

    gpt_standard_messages = [{"role": "system",
                                   "content": f"You're a assistant robot for senior in South Korea. Your name is 명자. Your being purpose is support.  So Please answer politely in korean and under 5 seconds. And if user say nothing then please do not say anything. and also analyze feeling of patient's sentence in one word. please add the result of feeling as a one word inside () on last sentence and answer korean. You can use 슬픔, 평범, 당황, 분노 word when you analyze the emotion of answer. Your patient's name is {nameValue} and {manWomanValue} is an old korean."}]

    def set_memory_size(self, memory_size):
        '''
        클래스 내의 질문 담을 메모리 저장
        :param memory_size: 메모리
        :return:
        '''
        self.memory_size = memory_size

    def gpt_send_anw(self, question: str):

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.gpt_standard_messages,
            temperature=0.8
        )

        answer = response['choices'][0]['message']['content']

        temp = 0
        ans_list = list(answer)

        if len(ans_list) > 4:

            for i in range(0, len(ans_list)):
                if ans_list[i] == '(':
                    temp = i

            ans_emo = []
            ans_re = []

            for k in range(temp + 1, temp + 3):
                ans_emo.append(ans_list[k])
            ans_emotion = "".join(ans_emo)

            for m in range(0, temp):
                ans_re.append(ans_list[m])
            ans_real = "".join(ans_re)
        else:
            ans_emotion = "평범"
            ans_real = ""

        self.gpt_standard_messages.append({"role": "user", "content": question})
        self.gpt_standard_messages.append({"role": "assistant", "content": answer})

        return [ans_emotion, ans_real]


def speak_first():
    '''
    먼저 말 거는 함수
    :return: X
    '''
    from time import localtime
    import random

    tm = localtime()

    question_list = ["오늘 몸 상태는 어때요?", "저는 오늘도 행복해요. 오늘 어떠세요??", "", "", "", "", "", "", "", ""]

    if tm.tm_hour == 7 and tm.tm_min == 00:
        speaking("좋은 아침이에요!! 오늘도 좋은 하루 되세요!!")
    elif tm.tm_hour == 22 and tm.tm_min == 00:
        speaking("이제 잘 시간이에요!! 편안한 밤 되세요!!")
    elif tm.tm_hour == 12 and tm.tm_min == 00:
        speaking("혈압약 드실 시간이에요!")
    elif 7 < tm.tm_hour < 22 and tm.tm_min == 00:
        random_ans = random.randrange(0, 9)
        if tm.tm_min == 30:
            if random_ans == 1:
                # 말 걸 내용들
                speaking(question_list[0])
            elif random_ans == 2:
                # 말 걸 내용들
                speaking(question_list[1])
    elif tm.tm_hour == 15 and tm.tm_min == 00:
        speaking("우리 산책 나가요!")

    time.sleep(3)


def speaking(anw_text):
    import os
    import urllib.request
    from pydub import AudioSegment
    from playsound import playsound as pl

    # NAVER CLOVA
    client_id = client_id_g
    client_secret = client_secret_g
    encText = urllib.parse.quote(anw_text)
    data = f"speaker=ndain&volume=0&speed=0&pitch=0&format=mp3&text=" + encText
    urls = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts"
    requests = urllib.request.Request(urls)
    requests.add_header("X-NCP-APIGW-API-KEY-ID", client_id)
    requests.add_header("X-NCP-APIGW-API-KEY", client_secret)
    response = urllib.request.urlopen(requests, data=data.encode('utf-8'))
    rescodes = response.getcode()
    if (rescodes == 200):
        print("mp3 저장 완료")
        response_body = response.read()
        with open('./ResultMP3.mp3', 'wb') as f:
            f.write(response_body)

        # 스피커 출력
        filename = "ResultMP3.mp3"
        dst = "test.wav"
        sound = AudioSegment.from_mp3(filename)
        sound.export(dst, format="wav")

        # data, fs = sf.read(filename, dtype='')
        pl("test.wav")

        # 제작된 음성 파일 삭제
        os.remove("ResultMP3.mp3")
        os.remove("test.wav")


def mic(time):
    '''
    주어진 음성을 마이크로 녹음 후 문장으로 변환
    :return: 음성 파일 문장으로 변환시켜 넘김
    '''
    import requests
    import sounddevice as sd
    from scipy.io.wavfile import write

    ## NAVER CLOVA API
    client_id = client_id_g
    client_secret = client_secret_g

    # 음성 녹음
    fs = 44100
    seconds = time

    myRecording = sd.rec(int(seconds * fs), samplerate=fs, channels=4)  # channels는 마이크 장치 번호
    print("녹음 시작")
    # 마이크 장치 번호 찾기 => python -m sounddevice
    sd.wait()
    write('sampleWav.wav', fs, myRecording)

    # Voice To Text => 목소리를 텍스트로 변환
    ## 기본 설정
    lang = "Kor"  # 언어 코드 ( Kor, Jpn, Eng, Chn )
    url = "https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=" + lang

    ## 녹음된 Voice 파일
    data_voice = open('sampleWav.wav', 'rb')

    ## 사용할 header
    headers = {
        "X-NCP-APIGW-API-KEY-ID": client_id,
        "X-NCP-APIGW-API-KEY": client_secret,
        "Content-Type": "application/octet-stream"
    }

    ## VTT 출력
    response = requests.post(url, data=data_voice, headers=headers)

    result_man = str(response.text)
    result = list(result_man)
    count_down = 0
    say_str = []

    for i in range(0, len(result) - 2):
        if count_down == 3:
            say_str.append(result[i])

        if response.text[i] == "\"":
            if count_down == 3:
                break
            else:
                count_down += 1

    anw_str = ''.join(map(str, say_str))

    print(anw_str)

    return anw_str


def name_check():
    import json
    global common
    common = 0
    with open('./user_value.json', 'r') as f:
        data = json.load(f)
        if data["user_name"] == "":
            use_sound("./mp3/first_0.wav")
            use_sound("./mp3/first_set_2.wav")
            name_ = mic(2)
            speaking(f"안녕하세요! {name_}님")
            use_sound("./mp3/first_set_3.wav")
            manWoman = mic(2)
            if manWoman == "남자":
                manWoman_ = "he"
            elif manWoman == "여자":
                manWoman_ = "she"
            else:
                while manWoman != "남자" or "여자":
                    use_sound("./mp3/first_set_4.wav")
                    manWoman = mic(2)
                    if manWoman == "남자":
                        manWoman_ = "he"
                        break
                    elif manWoman == "여자":
                        manWoman_ = "she"
                        break
            common = 1
            use_sound("./mp3/first_set_done.wav")
        else:
            name_ = data["user_name"]
            manWoman_ = data["user_value"]

        write_data = {
            "user_name": f"{name_}",
            "user_value": f"{manWoman_}"
        }

        if common == 1:
            with open('./user_value.json', 'w') as d:
                json.dump(write_data, d)

    return [name_, manWoman_]

def name_ini():
    import json
    write_data = {
            "user_name": "",
            "user_value": ""
        }
    with open('./user_value.json', 'w') as d:
                json.dump(write_data, d)

def use_sound(loc):
    from playsound import playsound as pl

    pl(loc)
