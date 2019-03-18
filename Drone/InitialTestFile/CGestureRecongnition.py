import numpy as np

class CGestureRecongnition:
    def __init__(self):
        self.MovementDirection = "Backward"

    def getMovement(self,image,movementcommand=0):
        #after gesture implementation movement command will be removed and movement will performded based on gesture
        if (movementcommand == 1):
            self.MovementDirection = "Backward"
        elif (movementcommand == 2):
            self.MovementDirection = "Forward"
        elif (movementcommand == 3):
            self.MovementDirection = "Left"
        elif (movementcommand == 4):
            self.MovementDirection = "Right"

        return self.MovementDirection