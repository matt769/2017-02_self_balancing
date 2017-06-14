
from msvcrt import getch
import serial

port = 'COM6'
baudrate = 115200
timeoutNum = 1
print "Settings can be configured in .py file"
print 'Current settings...'
print 'port:',port
print 'baudrate:',baudrate

# don't think this is perfect but intended to deal with situation where connection is already opon
try:
	ser = serial.Serial(port, baudrate, timeout=timeoutNum)
	print 'Connected. Go!'
except serial.SerialException:
	print "Error: Could not connect to",port
	print "Exiting program"
	exit()

lastKey = ''
while True:
	key = getch()
	# exit program with 'q'
	if key == 'q':
		print "Exiting program"
		break
	# receiver will handle any unexpected characters
	print 'New input:',key
	ser.write(key)
	lastKey = key
	# print 'Response from receiver:',ser.read(size=4)


try:
	#ser.write('x')
	print "Sending final stop command"
	ser.close()
	print "Closing connection"
except serial.SerialException:
	print "Connection lost"
	pass

print "Exiting program"
exit()