import json
from collections import deque

class RoverInterface:

    def __init__(self):
        self.instructionDeque = deque()

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
        self.instructionDeque.extend(instructions)
        print(self.instructionDeque)
