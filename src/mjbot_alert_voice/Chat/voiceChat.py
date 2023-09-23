import openai

## ChatGPT
openai.api_key = "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"


class MYOUNGJA():
    memory_size = 5
    nameValue = ""
    manWomanValue = ""

    def __init__(self, name, manWoman) -> None:
        MYOUNGJA.nameValue = name
        MYOUNGJA.manWomanValue = manWoman

    def set_memory_size(self, memory_size):
        '''
        클래스 내의 질문 담을 메모리 저장
        :param memory_size: 메모리
        :return:
        '''
        self.memory_size = memory_size

    def gpt_send_anw(self, question: str):
        '''
        질문을 ChatGPT에 넣어서 답변 출력
        :param question: 질문
        :return: 답변
        '''
        self.gpt_standard_messages = [{"role": "system",
                                       "content": f"You're a assistant robot for senior in South Korea. Your name is 명자. Your patient's name is {MYOUNGJA.nameValue} and {MYOUNGJA.manWomanValue} is an old korean. Your being purpose is support. So Please answer shortly, under 5 seconds , politely in korean. And if user say nothing then please do not say anything."}]
        self.gpt_standard_messages.append({"role": "user", "content": question})

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=self.gpt_standard_messages,
            temperature=0.8
        )

        answer = response['choices'][0]['message']['content']
        self.gpt_standard_messages.append({"role": "assistant", "content": answer})
        if self.memory_size * 2 < len(self.gpt_standard_messages):
            self.gpt_standard_messages.pop(1)
            self.gpt_standard_messages.pop(1)

        self.memory_size += 1

        return answer

    def mic(self):
        '''
        주어진 음성을 마이크로 녹음 후 문장으로 변환
        :return: 음성 파일 문장으로 변환시켜 넘김
        '''
        import requests
        import sounddevice as sd
        from scipy.io.wavfile import write

        ## NAVER CLOVA API
        client_id = "rlmmwycb6e"
        client_secret = "Vsprb6BSk7LP2IuWpSxdvvV5tGVriniDiHMXtodQ"

        # 음성 녹음
        fs = 44100
        seconds = 3

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

    def speaker(self, anw_text):
        '''
        주어진 말을 스피커를 통해 음성 변환 시켜 출력
        :param anw_text:
        :return: X
        '''
        import os
        import urllib.request
        from pydub import AudioSegment
        from playsound import playsound as pl

        # NAVER CLOVA
        client_id = "rlmmwycb6e"
        client_secret = "Vsprb6BSk7LP2IuWpSxdvvV5tGVriniDiHMXtodQ"

        encText = urllib.parse.quote(anw_text)
        data = "speaker=ndain&volume=0&speed=0&pitch=0&format=mp3&text=" + encText
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

    def speak_first(self):
        '''
        먼저 말 거는 함수
        :return: X
        '''
        from time import time, localtime
        import random

        tmt = time()
        tm = localtime(tmt)

        question_list = ["할머니 오늘 몸 상태는 어때요?", "저는 오늘도 행복해요. 할머니는 어때요??", "", "", "", "", "", "", "", ""]

        if tm.tm_hour == 7 and tm.tm_min == 00:
            self.speaker("좋은 아침이에요. 할머니!! 오늘도 좋은 하루 되세요!!")
        elif tm.tm_hour == 22 and tm.tm_min == 00:
            self.speaker("할머니 이제 잘 시간이에요!! 편안한 밤 되세요!!")
        elif tm.tm_hour == 12 and tm.tm_min == 00:
            self.speaker("할머니 혈압약 드실 시간이에요!")
        elif 7 < tm.tm_hour < 22 and tm.tm_min == 00:
            random_ans = random.randrange(0, 9)
            if tm.tm_min == 30:
                if random_ans == 1:
                    # 말 걸 내용들
                    self.speaker(question_list[0])
                elif random_ans == 2:
                    # 말 걸 내용들
                    self.speaker(question_list[1])
        elif tm.tm_hour == 15 and tm.tm_min == 00:
            self.speaker("할머니 우리 산책 나가요!")

    def findNegative(self, saySentence):
        '''
        문장 중 부정적 발화를 파악하는 함수. 어절 별로 나눠서 비교하는 방식
        :param saySentence:
        :return: X
        '''
        openai.api_key = "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"

        gpt_standard_messages = [{"role": "system",
                                    f"content": f"analyze feeling of {saySentence} in one word. please answer korean. You can use 기쁨, 슬픔, 평범, 당황, 분노, 사랑, 위험 word when you answer."}]
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=gpt_standard_messages,
            temperature=0.8,
        )

        gpt_standard_messages.append({"role": "user", "content": saySentence})
        answer = response['choices'][0]['message']['content']

        return answer

    def speaking(anw_text):
        '''
        주어진 말을 스피커를 통해 음성 변환 시켜 출력
        :param anw_text:
        :return: X
        '''
        import os
        import urllib.request
        from pydub import AudioSegment
        from playsound import playsound as pl

        # NAVER CLOVA
        client_id = "rlmmwycb6e"
        client_secret = "Vsprb6BSk7LP2IuWpSxdvvV5tGVriniDiHMXtodQ"
        encText = urllib.parse.quote(anw_text)
        data = "speaker=ndain&volume=0&speed=0&pitch=0&format=mp3&text=" + encText
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