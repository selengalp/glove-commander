from lib.vjoy.VirtualJoystick import *
from time import sleep
import serial

vjoy = VirtualJoystick(1)
ser = serial.Serial('COM5', 9600, timeout=1)

while True:
	if ser.inWaiting() > 0:
		sleep(0.005)
		line = ser.readline()
		#print line
		num = [int(s) for s in line.split('-') if s.isdigit()]
	
		y_axes = int(((180 - num[1]) * 364) + 16384)
		x_axes = int(((num[0] - 180) * 364) + 16384)
		vjoy.axe[Axis.Y].value = x_axes
		vjoy.axe[Axis.X].value = y_axes