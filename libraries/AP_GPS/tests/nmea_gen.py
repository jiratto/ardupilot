import serial
import time

ser = serial.Serial('/dev/ttyUSB0',38400)

try:
    while True:
        ser.write('$GPHDG,165.4,0.0,E,0.7,W*4D\r\n')
        ser.write('$GPMWV,145.9,R,3.4,N,A*24\r\n')
        ser.write('$GPMDA,,,,,29.7,C,,,,,,,,,,,,,,*0E\r\n')
        #ser.write('$GPVHW,,,165.4,M,,,,*3B\r\n')
        ser.write('$GPVHW,246,T,234,M,012.3,N,022.8,K*4F\r\n')
        ser.write('$SDDPT,3.6,0.0*52\r\n')
        time.sleep(1.0)

except KeyboardInterrupt:
    print("Ctrl-C pressed\n")
    ser.close()