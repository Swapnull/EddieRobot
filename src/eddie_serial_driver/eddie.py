#!/usr/bin/env python

import sys
import glob
import serial

class Eddie:
	def __init__(self):
		self.ser = getOpenPort()
		self.ser.baudrate = 115200


	def sendCommand(self, shape):
		if shape == 'triangle':
			s = 'GO 50 50\r'
			print s
			print 'going forward'
			self.ser.write(s.encode())

		elif shape == 'square':
			s = 'STOP 0\r'
			print s
			print 'stopping'
			self.ser.write(s.encode())

		elif shape == 'circle':
			s = 'go 50 D1\r'
			print s
			print ''
			self.ser.write(s.encode())

		elif shape == 'hexagon':
			s = 'GO D1 D1\r'
			print s
			self.ser.write(s.encode())

		# turn right
		elif shape == 'pentagon':
			s = 'GO D1 50\r'
			print s
			self.ser.write(s.encode())


#open a serial port
def getOpenPort():
	ports = getSerialPorts()
	choice = 0
	if len(ports) >  1:
		print "You have more than one serial port available: "
		for i in range(len(ports)):
			print "[", i , "]", ports[i]

		print "Which port would you like to use?: "
		choice =  int(raw_input())
	return serial.Serial(ports[choice])

# Lists serial ports
def getSerialPorts():
    if sys.platform.startswith('win'):
    	# windows
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
    	#linux/linux emulator
        #This excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
    	#osx
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

if __name__ == '__main__':
    eddie = Eddie()
    eddie.sendCommand('triangle')
