def googlemap_api():
    import googlemaps
    import serial
    from gps_set import longitude, latitude

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    API = "AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8"  # API 값

    # 위도 경도 -> 지번 주소로 변경 // 역지오코드
    gmaps = googlemaps.Client(key=API) # api key
    reverse_geocode_result = \
        gmaps.reverse_geocode((latitude, longitude), language='ko')
    gps = reverse_geocode_result[1]["formatted_address"]

    return gps

def send_message(problem):
    from twilio.rest import Client
    global problem_thing
    gps = googlemap_api()
    import time

    # Twilio
    # 계정 token 입력
    account_sid = "AC580149388d714abb2f99c81b120f4cea"
    auth_token = "b5c1267bea39194607fd4169018a4877"
    client = Client(account_sid, auth_token)

    if problem == 1:
        problem_thing = "낙상 사고"
    elif problem == 2:
        problem_thing = "화재 발생"

    message = client.messages \
        .create(
        body=f"위험 위험!!, 할머니가 위험해요. 종류 : {problem_thing}. 위치는 ",
        from_="+12294145018",
        to="+821025941808",
    )

    time.sleep(1000000)

# 실험용 코드, 주석 해제하고 돌려보세요.
# send_message(1)
