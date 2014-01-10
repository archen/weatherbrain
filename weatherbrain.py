from xbee import XBee
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)

xbee = XBee(ser)

# Continuously read and print packets
while True:
    try:
        response = xbee.wait_read_frame()
        print response
    except KeyboardInterrupt:
        break
        
ser.close()
