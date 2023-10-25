import os
import urllib.request
from pydub import AudioSegment
from playsound import playsound as pl

# NAVER CLOVA
client_id = "ud0o0y1iat"
client_secret = "eiQpNDsn5yTddyERg6U7s9IXXOSodlnD9UUMYq3k"
encText = urllib.parse.quote("네")
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

    pl("test.wav")
