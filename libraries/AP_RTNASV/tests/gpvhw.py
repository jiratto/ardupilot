#!/usr/bin/env python
import serial
import time
from datetime import datetime

with serial.Serial('/dev/ttyUSB1', 38400, timeout=1) as ser:
    while (True):
        time.sleep(1)
        sentence = b'$GPVHW,,,,,4.79,N,8.87,K*56\r\n'
        ser.write(sentence)
        now = datetime.now()
        now_str = now.strftime("%H:%M:%S")
        print(now_str + ' : ' + sentence)
