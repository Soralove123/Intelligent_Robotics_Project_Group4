class Delaytime:
    def __init__(self, robot):
        self.robot = robot
        self.current_time = 0
        self.last_time = 0
        self.wait_time = 0
        self.times = 0

    def Timer(self, wait_time):
        self.current_time = self.robot.getTime()
        self.wait_time = wait_time
        if self.current_time - self.last_time >= self.wait_time:
            print(f"已经用时: {self.current_time} 秒")
            self.last_time = self.current_time
    
    def Reset(self):
        self.times = 0
        
    def Delay(self, wait_time):
        timestep = int(self.robot.getBasicTimeStep())
        self.wait_time = wait_time
        self.last_time = self.robot.getTime()
        while self.robot.step(timestep) != -1:
             self.current_time = self.robot.getTime()
             if self.current_time - self.last_time >= self.wait_time:
                 break