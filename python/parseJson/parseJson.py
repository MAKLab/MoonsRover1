import json
import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 115200)
instr = open('instructions.json')
print instr
instructions = json.load(instr) #jsonloads for string rather than file!

while True:
	try:
		for x in instructions["ins"]:
			if x['command'] == 'forward':
				ser.write('MLF' + str(x['distance']) + ',')
				ser.write('MRF' + str(x['distance']) + ',')
				time.sleep(.5)
			elif x['command'] == 'back':
				ser.write('MLB' + str(x['distance']) + ',')
				ser.write('MRB' + str(x['distance']) + ',')
				time.sleep(.5)
			elif x['command'] == 'pan':
				ser.write('SP' + str(x['distance']) + ',')
			elif x['command'] == 'tilt':
				ser.write('ST' + str(x['distance']) + ',')	


		
	except (ValueError, KeyError, TypeError):
		print "JSON format error"

