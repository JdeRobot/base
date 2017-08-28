
class Transition():
    def __init__(self, id, destinationId, interfaces=None):
        self.id = id
        self.destinationId = destinationId
        self.interfaces = interfaces

    def init(self):
        pass

    def runCode(self):
        pass

    def checkCondition(self):
        return False
