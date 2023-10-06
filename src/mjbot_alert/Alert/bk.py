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

