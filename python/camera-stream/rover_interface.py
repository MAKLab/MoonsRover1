import json
import serial
import threading
import time

from collections import deque

class RoverInterface:

    # The instructions to parse
    instructionDeque = deque()

    # The thread that passes the instructions to the Arduino.
    thread = None

    # The serial port - currently does not exist/work
    ser = None
    
    # Make the class thread safe
    lock = threading.Lock()

    
    def initialize(self):
        try:
	    RoverInterface.serialPort = serial.Serial('/dev/ttyAMA0', 115200)
        
	except(OSError, serial.SerialException):
	    print("Caught SerialException")
	    RoverInterface.serialPort = None


    @classmethod
    def validCommands(self):
        return ['forward', 'back', 'left', 'right', 'pan', 'tilt']


    def validateInstructions(self, instructions):
        for instruction in instructions:
            try:
                if 'command' not in instruction:
                    return False

                if instruction['command'] not in RoverInterface.validCommands():
                    return False

                if 'distance' not in instruction:
                    return False

                # TODO: Validate the distance value

            except (ValueError, KeyError, TypeError):
                print("Failed to parse instruction: {}".format(instruction))
                return False

        return True



    def addValidInstructions(self, instructions):
        """Assumes the instructions are valid"""
        """Program will stop here until we can et access to variable"""  
        with lock:
        	n = len(cls.instructionDeque) 
        	RoverInterface.instructionDeque.extend(instructions)

        # If the thread is not running, create a new one and start it
        # TODO: is this safe? - Probably not
        if n == 0:
            RoverInterface.thread = threading.Thread(target=self._thread)
            RoverInterface.thread.start()


    @classmethod
    def _thread(cls):
        """This method is run by the thread. It runs the instructions on the Arduino."""
        while len(cls.instructionDeque) > 0:
            with lock:	
                instruction = cls.instructionDeque.popleft()
	    command = instruction['command']
	    if command == "forward":
	        cls.transmit("MLF255,")
	        cls.transmit("MRF255,")
	        time.sleep(1)
	    elif command == "back":
	        cls.transmit("MLB255,")
	        cls.transmit("MRB255,")
	        time.sleep(1)
            elif command == "right":
                cls.transmit("MLB255,")
		cls.transmit("MRF255,")
		time.sleep(1)
            elif command == "left":
                cls.transmit("MLF255,")
		cls.transmit("MRB255,")
		time.sleep(1)
	    elif command == "pan":
	        cls.transmit("SP0,")
		time.sleep(1)
            else:
                print("Command not handled: {}".format(command))
        
        # TODO: This is only reassigning the reference, not deleting the thread
        # cls.thread = None ; not needed? Thread ending anyway?


    @classmethod
    def transmit(self, command):
        if RoverInterface.serialPort is None:
	    print(command)
	    return
	
	RoverInterface.serialPort.write(command.encode('utf-8'))
