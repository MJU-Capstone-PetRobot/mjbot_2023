import googlemaps
from twilio.rest import Client

def googlemap_api():
    API = "AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8"  # API 값
    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API)  # api key
    # latitude, longitude = gps_set()
    
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

    if problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"명자 위험 알림. 종류 : {problem_thing}. 위치는 {gps}",
        from_="+18664707171",
        to="+821066860215"
    )
    print("메세지 전송 완료")