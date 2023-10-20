from gps import *
import serial, time, pynmea2
from datetime import datetime
import pytz, dateutil.parser

def gps_start():
    print("시작")
    """
    필요한 gpsd 라이브러리 설치
    sudo apt install gpsd,gpsd-clients
    pip3 install gpsd-py3
    pip3 install pynmea2
    """

    """
    gps 사용 시작
    sudo gpsd /dev/ttyFIQ0 -F /var/run/gpsd.sock
    sudo chmod 666 /dev/ttyFIQ0     >> 권한 설정
    sudo nano etcdefaultgpsd      >> gpsd 포트 지정
    -> DEVICES=devttyFIQ0 -> CTRL+O -> ENTER -> CTRL+Y
    sudo service gpsd restart
    sudo service gpsd status     >> make sure active(running)
    cgps
    """

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
    print("gps_start 끝")
def gps_set():
    print("gps_set 시작")
    # import the necessary libraries
    from gps import gps
    import time
    from datetime import datetime
    import pytz, dateutil.parser

    gps_start()

    running = True

    def getPositionData(gps):
        nx = gpsd.next()

        if nx['class'] == 'TPV':
            # extract gps info
            mju_latitude = getattr(nx, 'lat', "Unknown")
            mju_longitude = getattr(nx, 'lon', "Unknown")

            return [mju_latitude, mju_longitude]
        else:
            return [None, None]

    # Tell gpsd we are ready to receive messages
    gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)

    try:
        print("Application started!")
        while running:
            # call function to extract and append GPS data
            gps_data = getPositionData(gpsd)
            if gps_data[0] is not None and gps_data[1] is not None:
                latitude = gps_data[0]
                longitude = gps_data[1]
                return [latitude, longitude]
            # delay running the program for 1 second
            time.sleep(1)

    # if the user presses ctrl+c, the program will stop running
    except (KeyboardInterrupt):
        running = False
    print("gps set 끝")
def googlemap_api():
    import googlemaps
    import serial
    print("google map 시작")
    latitude = gps_set()[0]
    longitude = gps_set()[1]

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    API = "AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8"  # API 값

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API) # api key
    reverse_geocode_result = \
        gmaps.reverse_geocode((latitude, longitude), language='ko')
    gps = reverse_geocode_result[1]["formatted_address"]
    print("google map 끝")
    return gps

def send_message(problem):
    print("메세지 전송 시작")
    from twilio.rest import Client
    global problem_thing
    gps = googlemap_api()
    import time

    # Twilio
    # 계정 token 입력
    account_sid = "AC9750ecd4335fbb71324acf4bd9b64857"
    auth_token = "0b8f878c64425badea5a26bde51a4b80"
    client = Client(account_sid, auth_token)

    if problem == 1:
        problem_thing = "낙상 사고"
    elif problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"위험 위험!!, 할머니가 위험해요. 종류 : {problem_thing}. 위치는 {gps}",
        from_="+14437015330",
        to="+821095560116"
    )
    print("메세지 전송 끝")
    time.sleep(1000000)
    
