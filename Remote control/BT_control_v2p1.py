#v2 CHANGES
#just checks for any response

#TO ADD
# should i use serial.SerialException in except clause?
# add some code to both this and receiver (in terms of answer/response) to confirm all is as expected
# check if there's anything to receive before printing



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
	# don't send any more data to serial port if it's the same as before
	# this could be a problem if I want to add a timeout on the receiver side
	#if key == lastKey:
	#	continue
	# receiver will handle any unexpected characters
	print 'New input:',key
	ser.write(key)
	#ser.write('\n')
	lastKey = key
	# print 'Response from receiver:',ser.read(size=4)

# try sending kill command


try:
	ser.write('x')
	#ser.write('\n')
	print "Sending final stop command"
	ser.close()
	print "Closing connection"
except serial.SerialException:
	print "Connection lost"
	pass

print "Exiting program"
exit()