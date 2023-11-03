# import gps
# import serial
# import time
# import pynmea2
import googlemaps
from twilio.rest import Client
# from datetime import datetime


# def gps_set():
#     gpsd = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
#     print("gps 작동 시작")
#     while True:
#         # Wait for valid data
#         while True:
#             nx = gpsd.next()
#             if nx['class'] == 'TPV':
#                 break
#         # Extract gps info
#         mju_latitude = getattr(nx, 'lat', "Unknown")
#         mju_longitude = getattr(nx, 'lon', "Unknown")
#         if mju_latitude != "Unknown" and mju_longitude != "Unknown":
#             return [mju_latitude, mju_longitude]


def googlemap_api():
    API = "AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8"  # API 값
    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API)  # api key
    # latitude, longitude = gps_set()

    ## GPS가 잘 안 되어서, 학교 주소로 임의로 설정
    latitude = 37.57940
    longitude = 126.88975
    
    reverse_geocode_result = \
        gmaps.reverse_geocode((latitude, longitude), language='ko')
    gps = reverse_geocode_result[1]["formatted_address"]
    return gps


def send_message(problem):
    gps = googlemap_api()
    # Twilio
    # 계정 token 입력
    account_sid = "AC957a6e0d227e0a045846fc47c758f5e4"
    auth_token = "487fb07c4964fc01986ea143728fa75c"
    client = Client(account_sid, auth_token)
    problem_thing = ""

    if problem == 1:
        problem_thing = "낙상 사고"
    elif problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"명자 위험 알림. 종류 : {problem_thing}. 위치는 {gps}",
        from_="+18664707171",
        to="+821066860215"
    )
    print("메세지 전송 완료")


""" 여기 부분 예전 코드
def gps_start():
    필요한 gpsd 라이브러리 설치
    sudo apt install gpsd,gpsd-clients
    pip3 install gpsd-py3
    pip3 install pynmea2

    gps 사용 시작
    sudo gpsd /dev/ttyFIQ0 -F /var/run/gpsd.sock
    sudo chmod 666 /dev/ttyFIQ0     >> 권한 설정
    sudo nano etcdefaultgpsd      >> gpsd 포트 지정
    -> DEVICES=devttyFIQ0 -> CTRL+O -> ENTER -> CTRL+Y
    sudo service gpsd restart
    sudo service gpsd status     >> make sure active(running)
    cgps

    # 라즈베리파이 시리얼 통신
    port = '/dev/ttyFIQ0'
    baud = 9600
    serialPort = serial.Serial(port, baudrate=baud, timeout=0.5)

    #while True:
    str = serialPort.readline().decode().strip()
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        mj_latitude = round(msg.latitude, 6)
        mj_longitude = round(msg.longitude, 6)
    time.sleep(0.01)
"""