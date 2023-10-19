import serial,time,pynmea2

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

# 라즈베리파이 시리얼 통신
port = '/dev/ttyACM0'
baud = 9600
serialPort = serial.Serial(port, baudrate = baud, timeout = 0.5)

while True:
    str = serialPort.readline().decode().strip()
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        mj_latitude = round(msg.latitude, 6)
        mj_longitude = round(msg.longitude, 6)
    time.sleep(0.01)