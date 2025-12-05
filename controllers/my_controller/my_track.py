import motor_controller as M_C

class Tracker:
    def __init__(self, robot):
        self.robot = robot
        self.tracker_front_left = None
        self.tracker_front_middle = None
        self.tracker_front_right = None
        self.motor_controller = M_C.MotorController(self.robot)
        
        self.last_error = 0
        self.integral = 0
        self.last_state = 0


    def Init_ALL_Distance_Sensor_And_Motors(self):
        self.tracker_front_left = self.robot.getDevice("Distance_Sensor_Front_Left")
        self.tracker_front_middle = self.robot.getDevice("Distance_Sensor_Front_Middle")
        self.tracker_front_right = self.robot.getDevice("Distance_Sensor_Front_Right")
        self.motor_controller.Init_All_Motors()
        self.motor_controller.Stop()


    def Enable_Distance_sensor(self, timestep):
        self.tracker_front_left.enable(timestep)
        self.tracker_front_middle.enable(timestep)
        self.tracker_front_right.enable(timestep)

    def Track_Way(self, v):
        self.tracker_front_left_value = self.tracker_front_left.getValue()
        self.tracker_front_middle_value = self.tracker_front_middle.getValue()
        self.tracker_front_right_value = self.tracker_front_right.getValue()


        diff = self.tracker_front_right_value - self.tracker_front_left_value

        if diff >= 50:
            if self.last_state == -1:
                self.motor_controller.Turn_Left(v)
                self.last_state = 0
            else:
                self.motor_controller.Turn_Right(v)
                self.last_state = 1
        elif diff == 0 and self.last_state == 0:
            self.motor_controller.Move_Forward(v)
            self.last_state = 0
        elif diff <= -50:
            if self.last_state == 1:
                self.motor_controller.Turn_Right(v)
                self.last_state = 0
            else:
                self.motor_controller.Turn_Left(v)
                self.last_state = -1
        else:
            self.motor_controller.Move_Forward(v)


          