from gui.codegen.transition import Transition
from time import time

class TemporalTransition(Transition):
    def __init__(self, id, destinationId, elapsedTime):
        super().__init__(id, destinationId)
        # elapsed time in milliseconds
        self.elapsedTime = elapsedTime
        self.startTime = None

    def init(self):
        self.startTime = time()*1000

    def runCode(self):
        pass

    def checkCondition(self):
        super().checkCondition()
        diffTime = (time()*1000)-self.startTime
        if diffTime > self.elapsedTime:
            return True
        else:
            return False
