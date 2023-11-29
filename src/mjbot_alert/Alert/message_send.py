import googlemaps
from twilio.rest import Client
from dotenv import load_dotenv
import os
load_dotenv()

def googlemap_api():
    import json
    API = os.getenv("GOOGLE_API") # API 값
    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API)  # api key
    with open('user_data/user_gps.json', 'r') as f:
        data = json.load(f)
        if data["lat"] == "0.000000" or data["lon"] == "0.000000":
            latitude = "37.2227"
            longitude = "127.1902"
        else:
            latitude = data["lat"]
            longitude = data["lon"]
    
    reverse_geocode_result = \
        gmaps.reverse_geocode((latitude, longitude), language='ko')
    gps = reverse_geocode_result[0]["formatted_address"]
    return gps


def send_message(problem):
    import json
    gps = googlemap_api()
    # Twilio
    # 계정 token 입력
    account_sid = os.getenv("TWILIO_SID")
    auth_token =  os.getenv("TWILIO_TOKEN")
    client = Client(account_sid, auth_token)
    problem_thing = ""

    if problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"명자 위험 알림. 종류 : {problem_thing}. 위치는 {gps}",
        from_= os.getenv("FROM_TWILIO"),
        to= os.getenv("TO_TWILIO")
    )
    print("메세지 전송 완료")

    write_data = {
            "danger": "off"
        }
    
    with open('user_data/user_danger.json', 'w') as f:
            json.dump(write_data, f)
    
