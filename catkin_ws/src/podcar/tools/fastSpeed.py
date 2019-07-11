import serial
import time

port = '/dev/ttyArduino'
#port = '/dev/ttyACM2'

print("only works with dead mans handle\n")
print("Starting the program\n")

ser = serial.Serial(port, 115200)

print("Opened the communication with a port\n")
#protocol should return two lines of hello
line = ser.readline()
print(line)


if 1:
	lineout = "FA:245\r\n"   #140 is stop.  240 is fast fwd

	cmd = list(lineout)
	for char in cmd:
		ser.write(char.encode())

	linein = ser.readline()
	print(linein)
	#time.sleep(10)


if 0:
	file = open('pos.txt')
	while 1:

		line = file.readline()
#		line=line.strip()
		print("SENT: " + str(line))
		if not line:
			break

#		ser.write(b"%s\r\n"%line)  #needs DOS style CR LF

		cmd = list(line)
		for char in cmd:
			ser.write(char.encode())

	
		line = ser.readline()
		print(line)
	
		time.sleep(5)
