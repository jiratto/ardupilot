import serial
import time

ser = serial.Serial('/dev/ttyUSB0',38400)

# _GPS_SENTENCE_HDG = 138,    // [6]  $GPHDG,165.4,0.0,E,0.7,W*4D
# _GPS_SENTENCE_ZDA = 148,    // [7]  $GPZDA,044910,04,01,2021,,*44
# _GPS_SENTENCE_MWD = 158,    // [9]  $GPMWD,311.6,T,,,3.4,N,1.7,M*34
# _GPS_SENTENCE_MDA = 178,    // [21] $GPMDA,,,,,29.7,C,,,,,,,,,,,,,,*0E
# _GPS_SENTENCE_MWV = 208,    // [6]  $GPMWV,145.9,R,3.4,N,A*24
# _GPS_SENTENCE_VHW = 218,    // [9]  $GPVHW,,,165.4,M,,,,*3B

try:
    while True:
        ser.write('$GPHDG,165.4,0.0,E,0.7,W*4D\r\n')
        ser.write('$GPMWV,145.9,R,3.4,N,A*24\r\n')
        ser.write('$GPMDA,,,,,29.7,C,,,,,,,,,,,,,,*0E\r\n')
        time.sleep(1.0)

except KeyboardInterrupt:
    print("Ctrl-C pressed\n")
    ser.close()