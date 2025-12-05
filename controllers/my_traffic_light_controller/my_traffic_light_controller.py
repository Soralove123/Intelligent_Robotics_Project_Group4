"""my_traffic_light_controller controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

red_time = 9000
yellow_time = 3000
green_time = 6000

my_traffic_light_red = robot.getDevice("led_red")
my_traffic_light_yellow = robot.getDevice("led_yellow")
my_traffic_light_green = robot.getDevice("led_green")

my_traffic_light_red.set(0)
my_traffic_light_yellow.set(0)
my_traffic_light_green.set(0)

def rule_of_traffic(red_time, yellow_time, green_time):
    my_traffic_light_red.set(1)
    my_traffic_light_green.set(0)
    my_traffic_light_yellow.set(0)
    robot.step(red_time)
    my_traffic_light_green.set(1)
    my_traffic_light_yellow.set(0)
    my_traffic_light_red.set(0)
    robot.step(green_time)
    my_traffic_light_yellow.set(1)
    my_traffic_light_red.set(0)
    my_traffic_light_green.set(0)
    robot.step(yellow_time)

if __name__ == "__main__":
    while robot.step(timestep) != -1:
        rule_of_traffic(red_time, yellow_time, green_time)

