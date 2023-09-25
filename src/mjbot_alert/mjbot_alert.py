#!/usr/bin/env python
import os
from Alert.messageSending import *

#화재 감지 후, 메세지 송신
fire = fire_check()
if fire == 2:
    send_message(fire)
