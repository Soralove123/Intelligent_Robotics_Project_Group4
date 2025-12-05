import my_led as M_L

class MotorController:
    def __init__(self, robot):
        self.robot = robot
        self.Motor_Front_Left = None
        self.Motor_Front_Right = None
        self.Motor_Back_Left = None
        self.Motor_Back_Right = None
        self.led = M_L.LedController(self.robot)

    def Init_All_Motors(self):
      self.Motor_Front_Left = self.robot.getDevice('Motor_Front_Left')
      self.Motor_Front_Right = self.robot.getDevice('Motor_Front_Right')
      self.Motor_Back_Left = self.robot.getDevice('Motor_Back_Left')
      self.Motor_Back_Right = self.robot.getDevice('Motor_Back_Right')

      self.Motor_Front_Left.setPosition(float('inf'))
      self.Motor_Front_Right.setPosition(float('inf'))
      self.Motor_Back_Left.setPosition(float('inf'))
      self.Motor_Back_Right.setPosition(float('inf'))

      self.led.InitLed()
  
    def Move_Forward(self, v):
      self.Motor_Front_Left.setVelocity(v)
      self.Motor_Front_Right.setVelocity(v)
      self.Motor_Back_Left.setVelocity(v)
      self.Motor_Back_Right.setVelocity(v)
      self.led.GoingLight()
  
    def Turn_Left(self, v):
      self.Motor_Front_Left.setVelocity(-v)
      self.Motor_Front_Right.setVelocity(v)
      self.Motor_Back_Left.setVelocity(-v)
      self.Motor_Back_Right.setVelocity(v)
  
    def Turn_Right(self, v):
      self.Motor_Front_Left.setVelocity(v)
      self.Motor_Front_Right.setVelocity(-v)
      self.Motor_Back_Left.setVelocity(v)
      self.Motor_Back_Right.setVelocity(-v)
  
    def Move_Backward(self, v):
      self.Motor_Front_Left.setVelocity(-v)
      self.Motor_Front_Right.setVelocity(-v)
      self.Motor_Back_Left.setVelocity(-v)
      self.Motor_Back_Right.setVelocity(-v)
      
    def Stop(self):
      self.Motor_Front_Left.setVelocity(0)
      self.Motor_Front_Right.setVelocity(0)
      self.Motor_Back_Left.setVelocity(0)
      self.Motor_Back_Right.setVelocity(0)
      self.led.StopLight()
        
    def Set_Speed(self, left_speed, right_speed):
      self.Motor_Front_Left.setVelocity(left_speed)
      self.Motor_Front_Right.setVelocity(right_speed)
      self.Motor_Back_Left.setVelocity(left_speed)
      self.Motor_Back_Right.setVelocity(right_speed)