# from gps import *
# import time
# import json

# running = True



# def getPositionData(gps):
#     nx = gpsd.next()

#     if nx['class'] == 'TPV':
#         # extract gps info
#         latitude = getattr(nx, 'lat', "Unknown")
#         longitude = getattr(nx, 'lon', "Unknown")

#         file_path = "/home/pi/Desktop/GPS/gps_info.json"

#         data = {'gps': []}
#         data['gps'].append({
#             "latitude": f"{latitude}",
#             "longitude": f"{longitude}",
#         })

#         print(data)

#         with open(file_path, 'w') as outfile:
#             json.dump(data, outfile, indent=4)

#         # # csv 작성 코드
#         # f = open('all_gps_info.csv','a',newline='')
#         # wr = csv.writer(f)
#         # wr.writerow(data)
#         # f.close()

# # Tell gpsd we are ready to receive messages
# gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)

# try:
#     print("Application started!")
#     while running:
#         # call function to extract and append GPS data
#         getPositionData(gpsd)
#         # delay running the program for 1 second
#         time.sleep(1)

# # if the user presses ctrl+c, the program will stop running
# except (KeyboardInterrupt):
#     running = False