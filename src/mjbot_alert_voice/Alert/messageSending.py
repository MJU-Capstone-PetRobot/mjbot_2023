# from twilio.rest import Client
# import serial
# import time

# def googlemap_api():
#     import json
#     import googlemaps

#     # GPS 위치 가져오기
#     file_path = "/home/pi/Desktop/GPS/gps_info.json"
#     with open(file_path, "r") as json_file:
#         json_data = json.load(json_file)
#         latitude = json_data["gps"][0]["latitude"]
#         longitude = json_data["gps"][0]["longitude"]

#     # 위도 경도 -> 지번 주소로 변경 // 역지오코드

#     API = 'AIzaSyD5fvkrnY2xbyp7DB9LK-bQbT1RzbgpvE8'  # API 값

#     # 위도 경도 -> 지번 주소로 변경 // 역지오코드
#     gmaps = googlemaps.Client(key=API)  # api key
#     reverse_geocode_result = \
#         gmaps.reverse_geocode((latitude, longitude), language='ko')
#     gps = reverse_geocode_result[1]["formatted_address"]

#     return gps

# # 화재 감지
# def fire_check():
#     global fire
#     if __name__ == '__main__':
#         ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
#         ser.flush()

#         if ser.in_waiting > 0:
#             line = ser.readline().decode('utf-8').rstrip()
#             s_line = int(line)
#             if s_line > 200:
#                 fire = 2
#             else:
#                 fire = -1

#     return fire


# def send_message(problem):

#     # 위치 확인
#     global problem_thing
#     gps = googlemap_api()

#     # Twilio
#     # 계정 token 입력
#     account_sid =
#     auth_token =
#     client = Client(account_sid, auth_token)

#     if problem == 1:
#         problem_thing = "낙상 사고"
#     elif problem == 2:
#         problem_thing = "화재 발생"

#     message = client.messages \
#         .create(
#         body=f"위험 위험!!, 할머니가 위험해요. 종류 : {problem_thing}. 위치는 {gps}",
#         from_=""
#         to =""
#     )

#     time.sleep(1000000)
