from twilio.rest import Client
from gps import *
import time
import json
from datetime import datetime
import pytz, dateutil.parser

running = True

"""
필요한 gpsd 라이브러리 설치
sudo apt install gpsd,gpsd-clients
pip3 install gpsd-py3
"""

"""
gps 사용 시작
sudo systemctl stop gpsd
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock
"""

def getPositionData(gps):
    nx = gpsd.next()

    if nx['class'] == 'TPV':
        # extract gps info
        latitude = getattr(nx, 'lat', "Unknown")
        longitude = getattr(nx, 'lon', "Unknown")

        return [latitude, longitude]

def googlemap_api():
    import googlemaps
    gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
    latitude = getPositionData(gpsd)[0]
    longitude = getPositionData(gpsd)[1]

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드

    API = 'AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8'  # API 값

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API)  # api key
    reverse_geocode_result = \
        gmaps.reverse_geocode((latitude, longitude), language='ko')
    gps = reverse_geocode_result[1]["formatted_address"]

    return gps


def send_message(problem):
    # 위치 확인
    global problem_thing
    gps = googlemap_api()

    # Twilio
    # 계정 token 입력
    account_sid = "AC1284d2af3f21aea53a0c14979072d54e"
    auth_token = "b21584709bd00e56cb0738fbe814d99c"
    client = Client(account_sid, auth_token)

    if problem == 1:
        problem_thing = "낙상 사고"
    elif problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"위험 위험!!, 할머니가 위험해요. 종류 : {problem_thing}. 위치는 {gps}",
        from_="+16562186034",
        to="+821085930557",
    )

    time.sleep(1000000)
