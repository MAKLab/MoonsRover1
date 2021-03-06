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


    def initialize(self):
        try:
            RoverInterface.serialPort = serial.Serial('/dev/ttyAMA0', 115200)
        except(OSError, serial.SerialException):
            print("Caught SerialException")
            RoverInterface.serialPort = None


    def status(self):
        """The rover status as a JSON object"""
        return {'hasSerial': RoverInterface.serialPort is not None}


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
        RoverInterface.instructionDeque.extend(instructions)

        # If the thread is not running, create a new one and start it
        # TODO: is this safe? - Probably not
        if RoverInterface.thread is None:
            RoverInterface.thread = threading.Thread(target=self._thread)
            RoverInterface.thread.start()


    @classmethod
    def _thread(cls):
        """This method is run by the thread. It runs the instructions on the Arduino."""
        while len(cls.instructionDeque) > 0:
            instruction = cls.instructionDeque.popleft()
	    command = instruction['command']
	    distance = instruction['distance']
	    if command == "forward":
	        cls.transmit("MAF{},".format(distance))
	    elif command == "back":
	        cls.transmit("MAB{},".format(distance))
            elif command == "right":
	        cls.transmit("MAR{},".format(distance))
            elif command == "left":
	        cls.transmit("MAL{},".format(distance))
	    elif command == "pan":
	        cls.transmit("SP{},".format(distance))
	    elif command == "tilt":
	        cls.transmit("ST{},".format(distance))
	    else:
	        print("Command not handled: {}".format(command))

        # TODO: This is only reassigning the reference, not deleting the thread
        cls.thread = None


    @classmethod
    def transmit(self, command):
        if RoverInterface.serialPort is None:
	    print(command)
	    return

	RoverInterface.serialPort.write(command.encode('utf-8'))
