class LedController:
    def __init__(self, robot):
        self.robot = robot
        self.Led_Middle = None
        self.Led_Left = None
        self.Led_Right = None
        self.blink_state_left = False
        self.blink_state_right = False

    def InitLed(self):
        self.Led_Middle = self.robot.getDevice('led')
        self.Led_Left = self.robot.getDevice('led_left')
        self.Led_Right = self.robot.getDevice('led_right')
        self.Led_Middle.set(0)
        self.Led_Left.set(0)
        self.Led_Right.set(0)

    def PopLight(self, function):
        if function == "Left":

            self.blink_state_left = not self.blink_state_left
            if self.blink_state_right == True:
                self.Led_Left.set(2)
            elif self.blink_state_right == False:
                self.Led_Left.set(0)

        if function == "Right":

            self.blink_state_right = not self.blink_state_right
            if self.blink_state_right == True:
                self.Led_Right.set(2)
            elif self.blink_state_right == False:
                self.Led_Right.set(0)

    def GoingLight(self):
        self.Led_Middle.set(2)
        self.Led_Right.set(0)
        self.Led_Left.set(0)

    def StopLight(self):
        self.Led_Middle.set(1)
        self.Led_Right.set(0)
        self.Led_Left.set(0)

    def TurnLeftLight(self):
        self.Led_Middle.set(2)
        self.PopLight('Left')
        self.robot.step(200)
        self.Led_Right.set(0)

    def TurnRightLight(self):
        self.Led_Middle.set(2)
        self.Led_Left.set(0)
        self.PopLight('Right')
        self.robot.step(200)
