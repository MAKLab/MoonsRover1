import json
import serial

ser = serial.Serial('/dev/ttyAMA0', 115200)

instructions = json.loads(open("instructions.json")

try:
	for x in instructions['sequence']
		print x['move']
		ser.write(x['move'] + ',')
		print x['pan']
		ser.write(x['pan'] + ',') #try this
		print x['tilt']
		ser.write(x['tilt'] + ',') #try this
		
except (ValueError, KeyError, TypeError):
	print "JSON format error"

