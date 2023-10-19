# import the necessary libraries
from gps import *
import time
from datetime import datetime
import pytz, dateutil.parser

running = True


def getPositionData(gps):
    nx = gpsd.next()

    if nx['class'] == 'TPV':
        # extract gps info
        mju_latitude = getattr(nx, 'lat', "Unknown")
        mju_longitude = getattr(nx, 'lon', "Unknown")

        return [mju_latitude, mju_longitude]


# Tell gpsd we are ready to receive messages
gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)

try:
    print("Application started!")
    while running:
        # call function to extract and append GPS data
        latitude = getPositionData(gpsd)[0]
        longitude = getPositionData(gpsd)[1]
        # delay running the program for 1 second
        time.sleep(1)

# if the user presses ctrl+c, the program will stop running
except (KeyboardInterrupt):
    running = False
