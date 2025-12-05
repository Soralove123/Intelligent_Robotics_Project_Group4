"""my_controller controller."""

from controller import Supervisor
from controller import Receiver
from controller import Keyboard
from controller import Emitter
import motor_controller as M_C
import numpy as np
import my_timedelay as M_D
import my_led as M_LED
import my_track as M_T
import my_selection as M_S
import json
import math
import re
import os
import my_camera as M_CA

# create the Robot instance.
robot = Supervisor()
keyboard = Keyboard()

# set velocity
v = 10
base_speed = 10

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable keyboard
keyboard.enable(timestep)

# init current time
current_time = 0
# set wait time
wait_time = 5

# gps
my_gps = robot.getDevice("robot_gps")
my_gps.enable(timestep)

my_destination_map = {
    "Loading Start" : (2.19, -9.17),
    "Loading Bay A" : (-1.41, -8.78),
    "Loading Bay B" : (-3.14, -8.71),
    "Loading Bay C" : (-4.83, -5.55),
    "Loading Bay D" : (0.16, -8.71)
}

my_traffic_light = [(-0.06, -11.93),
                    ]


# Approach the target range
my_goal_radius = 0.6
my_home_radius = 0.3

# init robot emitter
robot_emitter = robot.getDevice("robot_emitter")

# init robot receiver
my_receiver = robot.getDevice("robot_receiver")
my_receiver.enable(timestep)

# init all motors
motor_controller = M_C.MotorController(robot)
motor_controller.Init_All_Motors()
motor_controller.Stop()

# init camera
camera = M_CA.MyCamera(robot, keyboard)
camera.Init_All_Module()

# init selection model
my_select = M_S.MySelection(robot, keyboard)
my_select.Init_ALL_Modules()

my_imu = robot.getDevice("inertial unit")
my_imu.enable(timestep)
USE_IMU = True


# init my_tracker
my_tracker = M_T.Tracker(robot)
my_tracker.Init_ALL_Distance_Sensor_And_Motors()
my_tracker.Enable_Distance_sensor(timestep)
tracker_left_middle = robot.getDevice("Distance_Sensor_Left_Middle")
tracker_right_middle = robot.getDevice("Distance_Sensor_Right_Middle")
tracker_front_middle = robot.getDevice("Distance_Sensor_Front_Middle")
tracker_front_left = robot.getDevice("Distance_Sensor_Front_Left")
tracker_front_right = robot.getDevice("Distance_Sensor_Front_Right")

tracker_left_middle.enable(timestep)
tracker_right_middle.enable(timestep)
tracker_front_middle.enable(timestep)
tracker_front_left.enable(timestep)
tracker_front_right.enable(timestep)

tracker_left_middle_value = 0
tracker_right_middle_value = 0

delay = M_D.Delaytime(robot)
led_controller = M_LED.LedController(robot)
led_controller.InitLed()

current_order = None
order_data = None
completed_orders = []
robot_order = 0
start_yaw = None
print(f"The initialization of the order transportation robot has been completed\n")

# The menu
def menu():
    print("Please select the function\n")
    print("1 Learning mode - Assessment method\n")
    print("2 Deploy mode - Assessment method\n")
    print("3 Learning mode - Q_Learning\n")
    print("4 Deploy mode - Q_Learning\n")

# Generate the feedback
def generate_feedback(time, destination, status):
    feedback_data = {
        "timestamp" : time,
        "destination" : destination,
        "order_status" : status
    }
    return feedback_data

# Feedback of the order situation
def send_feedback(feedback_data):
    payload = json.dumps(feedback_data).encode('utf-8')
    robot_emitter.send(payload)
    print(f"Feedback has been sent\n")

# Receive the order
def order_incoming(order_data):
    print(f"The robot received a new order: ID#{order_data['order_id']}")
    print(f"Target: {order_data['destination']}")
    destination = order_data['destination']
    location_in_map = my_destination_map[destination]
    print(f"Destination of the target:{location_in_map}")
    current_order = order_data
    return location_in_map, destination

# Order completed feed back
def complete_order(current_order):
    if current_order:
        current_order['status'] = 'completed'
        current_order['completion_time'] = robot.getTime()
        completed_orders.append(current_order)
        print(f"Order completed: ID#{current_order['order_id']}")
        current_order = None
        robot_order = 0
        return robot_order

# Receive the massages
def check_receiver():
    if my_receiver.getQueueLength() > 0:
        data = my_receiver.getString()
        try:
            order_data = json.loads(data)
            location, destination = order_incoming(order_data)
            my_receiver.nextPacket()
            return location, destination
        except json.JSONDecodeError:
            print(f"Receive the massage: {data}\n")
            my_receiver.nextPacket()
    return None

# Find the robot location
def get_robot_location():
    x,y,z = my_gps.getValues()
    return (x, y)

# Calculate the distance
def judge_distance(x, y):
    robot_location = get_robot_location()
    distance = math.sqrt(math.pow((robot_location[0] - x), 2) + math.pow((robot_location[1] - y), 2))
    return distance

# Evaluation plan
def score(time, choice, send_complete, back_complete, error):
    score = 0
    if send_complete:
        score += 300
    if back_complete:
        score += 300
    score -= choice * 2
    score -= time * 0.3
    score -= error * 20
    return score

# Standardization perspective
def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# Get the robot yaw
def get_yaw():
    if my_imu is not None:
        roll, pitch, yaw = my_imu.getRollPitchYaw()
        return yaw
    else:
        return None

# Aligned to the start orientation for a new round
def rotate_to_head(target_yaw, v, error = 0.03):
    if target_yaw is None:
        print("The starting orientation was not recorded. Skip alignment\n")
        return

    while robot.step(timestep) != -1:
        yaw = get_yaw()
        if yaw is None:
            print("The orientation cannot be read and alignment is terminated\n")
            break

        err = wrap_pi(target_yaw - yaw)

        if abs(err) < error:
            motor_controller.Stop()
            print(f"Aligned to the initial orientation, the error is {err} rad\n")
            break

        if err > 0:
            motor_controller.Turn_Left(v)

        else:
            motor_controller.Turn_Right(v)

# Check the learning times
def check_learn_times(file_path, target_name: str, need_times: int = 10):
    count = 0
    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith(target_name + " Decision"):
                count += 1
            elif line.startswith(target_name) == None:
                count = 0

    if count >= need_times:
        print(f"{target_name} has completed {need_times} learning. Current number: {count}\n")
        return 0
    else:
        print(f"{target_name} has not completed {need_times} learning. Current number: {count}, still lacking {need_times -count} learning\n")
        return need_times - count

# Reset the robot position to start location
def reset_robot_pose_to_start(start_pos, start_yaw=None):
    motor_controller.Stop()
    robot.step(timestep)

    robot_node = robot.getSelf()
    trans_field = robot_node.getField("translation")
    rot_field = robot_node.getField("rotation")

    x = start_pos[0]
    y = start_pos[1]
    z = START_Z

    trans_field.setSFVec3f([x, y, z])

    if start_yaw is not None:
        rot_field.setSFRotation([0, 0, 1, start_yaw])

    robot_node.setVelocity([0, 0, 0, 0, 0, 0])

    motor_controller.Stop()

def compute_speed_with_traffic(base_speed, my_traffic_light, camera, my_select, massage_time):
    for traffic_light in my_traffic_light:
        traffic_light_distance = judge_distance(traffic_light[0], traffic_light[1])

        v = base_speed
        time = massage_time
        if 1.4 >= traffic_light_distance > 0.9:
            traffic_color = camera.Run_Traffic_Light_Camera()
            if traffic_color == "RED":
                v = base_speed / 5.0
            elif traffic_color == "YELLOW":
                v = base_speed / 2.0
            elif traffic_color == "GREEN":
                v = base_speed
            else:
                v = base_speed

        elif traffic_light_distance <= 0.9 and traffic_light_distance >= 0.8:
            traffic_color = camera.Run_Traffic_Light_Camera()
            if traffic_color == "RED":
                if my_select.traffic_done == 1:
                    v = base_speed
                    if time == 0:
                        print("The Choice has been done before red\n")
                        time = 1
                    return v, time
                else:
                    v = 0.0
                    if time == 0:
                        print("The light of injunction is red, STOP!\n")
                        time = 1
                    return v, time
            else:
                v = base_speed
                print(f"The light of injunction is green, GO\n")
                time = 0

        else:
            v = base_speed

    return v, time

# main function
if __name__ == "__main__":
    fail_fps = 0
    stuck_fps = 0
    order = 0
    location = (0, 0)
    start_yaw = get_yaw()
    if start_yaw is not None:
        print(f"Start orientation :{start_yaw}\n")
    start_sending_time = 0
    end_time = 0
    total_time = 0
    my_score = 0
    send_complete = 0
    back_complete = 0
    tracker_left_middle_value = tracker_left_middle.getValue()
    tracker_right_middle_value = tracker_right_middle.getValue()
    tracker_front_middle_value = tracker_front_middle.getValue()
    command = "Waiting"
    destination = None
    feedback_times = 0
    error = 0
    protect = 0
    start_pos = (2.19, -9.17)
    menu()
    decisions = []
    robot_node = robot.getSelf()
    trans_field = robot_node.getField("translation")
    initial_trans = trans_field.getSFVec3f()
    START_Z = initial_trans[2]
    fail_times = 0
    massage_time = 0
    while robot.step(timestep) != -1:
        key = keyboard.getKey()

        if key == ord('1'):
            print(f"The assessment learning mode has been selected\n")
            file_path = "learning_decision.txt"
            Start_location = my_destination_map['Loading Start']
            for location in my_destination_map:
                if location == 'Loading Start':
                    continue
                count = 0
                destination = my_destination_map[location]
                need_learning_times = check_learn_times(file_path, location, need_times=10)
                if count < need_learning_times:
                    print(f"The learning of the target {location} is underway\n")
                while robot.step(timestep) != -1:
                    tracker_front_middle_value = tracker_front_middle.getValue()
                    tracker_left_middle_value = tracker_left_middle.getValue()
                    tracker_right_middle_value = tracker_right_middle.getValue()

                    if count >= need_learning_times:
                        print(f"The number of study sessions has reached\n")
                        break

                    if my_select.check_stuck():
                        stuck_fps += 1
                    else:
                        stuck_fps = 0

                    if tracker_front_middle_value < 500 and tracker_left_middle_value < 500 and tracker_right_middle_value < 500:
                        fail_fps += 1
                    else:
                        fail_fps = 0

                    if fail_fps > 10:
                        protect = 1
                        print(f"Beyond the road\n")

                    if stuck_fps > 10:
                        protect = 1
                        print(f"The robot is stuck\n")

                    if protect == 1:
                        if fail_times >= 10:
                            print(f"Fail too many times, restart\n")
                            fail_times = 0
                            reset_robot_pose_to_start(start_pos, start_yaw)
                            break
                        print("This round of learning failed (on the way back), learning the current target again\n")
                        reset_robot_pose_to_start(start_pos, start_yaw)
                        my_select.Learning_Fail_Score_Learning()
                        my_select = M_S.MySelection(robot, keyboard)
                        my_select.Init_ALL_Modules()
                        command = 'Waiting'
                        order = 0
                        protect = 0
                        start_sending_time = 0
                        end_time = 0
                        total_time = 0
                        my_score = 0
                        send_complete = 0
                        back_complete = 0
                        fail_times += 1
                        count -= 1
                        continue
                    if count < need_learning_times:
                        match command:
                            case "Sending":
                                beginning = judge_distance(Start_location[0], Start_location[1])
                                if beginning <= 1.5:
                                    my_tracker.Track_Way(v)
                                    continue
                                my_select.Run_Score_Learning(v)
                                distance = judge_distance(my_destination_map[location][0], my_destination_map[location][1])

                                if distance <= my_goal_radius:
                                    print(f"Order completed\n")
                                    send_complete = 1
                                    command = "Go back"
                                    continue

                            case "Go back":
                                distance = judge_distance(Start_location[0], Start_location[1])
                                motor_controller.Stop()
                                my_select.Run_Score_Learning(v)

                                if distance <= 0.9:
                                    my_tracker.Track_Way(v)
                                    if distance <= my_home_radius:
                                        print(f"抵达站点\n")
                                        count += 1
                                        order = 0
                                        rotate_to_head(start_yaw, v)
                                        end_time = robot.getTime()
                                        total_time = end_time - start_sending_time
                                        choice = my_select.Report_Choice()
                                        back_complete = 1
                                        my_score = score(total_time, choice, send_complete, back_complete, error)
                                        with open(file_path, 'a+', encoding='utf-8') as f:
                                            f.write("End ")
                                            f.write(f"Score: {my_score}")
                                        command = "Waiting"
                                        continue
                                    continue

                            case "Waiting":
                                motor_controller.Stop()
                                my_select.cross_lock = False
                                print(order)
                                if order == 0:
                                    motor_controller.Stop()
                                    send_complete = 0
                                    back_complete = 0
                                    order += 1
                                    my_select.choice = 0
                                    with open(file_path, 'a+', encoding= 'utf-8') as f:
                                        f.write("\n")
                                        f.write(f'{location} Decisions: ')
                                    start_sending_time = robot.getTime()
                                    command = "Sending"
                                    print(f"location: {location}\n")
                                    print(f"destination: {destination}\n")
                                    continue

            print(f"All the targets have been learned\n")
            print(f"The best decisions are being generated\n")
            my_select.Deploy_Decision_Save()

        if key == ord('2'):
            print(f"The practical model for assessment learning has been selected\nn")
            file_path = "deploy_decision.txt"
            results = []

            location_re = re.compile(r"Location:\s*(.+)")
            decisions_re = re.compile(r"Best decision:\s*\[(.*?)\]")

            with open(file_path, "r", encoding='utf-8') as f:
                target_location = None
                for line in f:
                    line = line.strip()
                    if not line:
                        continue

                    my_location = location_re.match(line)
                    if my_location:
                        target_location = my_location.group(1).strip()
                        continue

                    my_decisions = decisions_re.match(line)
                    if my_decisions and target_location:
                        actions_line = my_decisions.group(1)
                        actions = [a.strip().strip("'\"") for a in actions_line.split(",") if a.strip()]
                        results.append({
                            "location": target_location,
                            "best_decision": actions
                        })
                        target_location = None

            Start_location = my_destination_map['Loading Start']
            while robot.step(timestep) != -1:

                match command:
                    case "Sending":
                        v, massage_time = compute_speed_with_traffic(base_speed, my_traffic_light, camera, my_select, massage_time)
                        beginning = judge_distance(Start_location[0], Start_location[1])
                        if beginning <= 1:
                            my_tracker.Track_Way(v)
                            continue
                        if feedback_times == 0:
                            feedback_data = generate_feedback(robot.getTime(), destination, command)
                            send_feedback(feedback_data)
                            feedback_times = 1
                        my_select.Run_Deploy_Score_Learning(v, decisions)
                        distance = judge_distance(location[0], location[1])

                        if distance <= my_goal_radius:
                            if feedback_times == 1:
                                feedback_data = generate_feedback(robot.getTime(), destination, "Complete")
                                send_feedback(feedback_data)
                                feedback_times = 0
                            order -= 1
                            print(f"Order completed\n")
                            location = None
                            command = "Go back"
                            continue

                    case "Go back":
                        v, massage_time = compute_speed_with_traffic(base_speed, my_traffic_light, camera, my_select, massage_time)
                        distance = judge_distance(Start_location[0], Start_location[1])
                        motor_controller.Stop()
                        if distance <= 1:
                            my_tracker.Track_Way(v)
                            if distance <= my_home_radius:
                                feedback_data = generate_feedback(robot.getTime(), destination, "Home")
                                send_feedback(feedback_data)
                                print(f"Arrive the Start Station\n")
                                rotate_to_head(start_yaw, v)
                                command = "Waiting"
                                continue
                            continue
                        my_select.Run_Deploy_Score_Learning(v, decisions)

                    case "Waiting":
                        my_select.Deploy_Decision_Save()
                        motor_controller.Stop()
                        my_select.choice = 0
                        my_select.cross_lock = False
                        print(f"Waiting for orders\n")
                        print(order)
                        if order == 0:
                            motor_controller.Stop()
                            order_result = check_receiver()
                            send_complete = 0
                            back_complete = 0
                            if order_result is not None:
                                location, destination = order_result
                                order += 1
                                start_sending_time = robot.getTime()
                                command = "Sending"
                                print(f"location: {location}\n")
                                print(f"destination: {destination}\n")
                                matched = False
                                for r in results:
                                    if destination == r['location']:
                                        decisions = r['best_decision']
                                        matched = True
                                        break
                                if not matched:
                                    print("No location was matched\n")
                                    exit()
                                continue

                        if location is None:
                            location = check_receiver()
                            motor_controller.Stop()
                            if location is None:
                                continue

        if key == ord("3"):
            start_back_time = 0
            robot_last_location = my_select.give_robot_last_location()
            robot_last_yaw = my_select.give_robot_last_yaw()
            print(f"The Q_Learning mode has been selected\n")
            file_path = "Q_Learning_times.txt"
            Start_location = my_destination_map['Loading Start']
            learning_times = {}

            if os.path.exists(file_path):
                with open(file_path, "r", encoding="utf-8") as f:
                    for line in f:
                        match = re.search(r"(.*?):\s*Learning Times:\s*(\d+)", line)
                        if match:
                            loc_name = match.group(1).strip()
                            cnt = int(match.group(2))
                            learning_times[loc_name] = cnt
            else:
                print("No path learning has started yet\n")

            for location in my_destination_map:
                if location == 'Loading Start':
                    continue

                count = learning_times.get(location, 0)

                destination = my_destination_map[location]
                need_learning_times = 10
                if count >= need_learning_times:
                    print(f"{location} Learning Times Max {need_learning_times}，skip this target\n")
                    continue
                else:
                    print(f"The learning of the target {location} is underway\n")

                while robot.step(timestep) != -1:
                    tracker_front_middle_value = tracker_front_middle.getValue()
                    tracker_left_middle_value = tracker_left_middle.getValue()
                    tracker_right_middle_value = tracker_right_middle.getValue()

                    if my_select.check_stuck():
                        stuck_fps += 1
                    else:
                        stuck_fps = 0

                    if tracker_front_middle_value < 500 and tracker_left_middle_value < 500 and tracker_right_middle_value < 500:
                        fail_fps += 1
                    else:
                        fail_fps = 0

                    if fail_fps > 10:
                        protect = 1
                        print(f"Beyond the road\n")

                    elif stuck_fps > 10:
                        protect = 1
                        print(f"Robot is stuck\n")

                    else:
                        robot_last_location = my_select.give_robot_last_location()
                        robot_last_yaw = my_select.give_robot_last_yaw()

                    if count >= need_learning_times:
                        print(f"{location} Learning Times Max {need_learning_times}，Switch Next Target\n")
                        break

                    match command:
                        case "Sending":
                            beginning = judge_distance(Start_location[0], Start_location[1])

                            if beginning <= 1.5:
                                my_tracker.Track_Way(v)
                                continue

                            my_select.Run_Learning_Q_Learning(v)
                            distance = judge_distance(my_destination_map[location][0],
                                                      my_destination_map[location][1])

                            if protect == 1:

                                # Prevent the webots robot from exploding due to too many resets
                                if fail_times >= 30:
                                    print(f"Fail too many times, restart\n")
                                    fail_times = 0
                                    reset_robot_pose_to_start(start_pos, start_yaw)
                                    break
                                print("Learning fail, back to last state and learn again\n")
                                reset_robot_pose_to_start(robot_last_location, robot_last_yaw)
                                fail_fps = 0
                                stuck_fps = 0
                                fail_times += 1
                                protect = 0
                                my_select.choice -= 1
                                my_select.state = 0
                                my_select.at_cross = 0
                                now = robot.getTime()
                                my_select._next_cross_check_time = now
                                my_select._last_pattern = 0
                                my_select._pattern_count = 0
                                my_select._last_cross_time = 0.0
                                my_select.choice_error = 1
                                continue

                            if distance <= my_goal_radius:
                                print(f"Order Completed\n")
                                send_complete = 1

                                end_time = robot.getTime()
                                total_time_send = end_time - start_sending_time
                                choice_send = my_select.Report_Choice()
                                back_complete = 0
                                error_flag = fail_times

                                send_score = score(total_time_send, choice_send,
                                                   send_complete, back_complete, error_flag)
                                fail_times = 0
                                my_select.Q_End_Episode(send_score)
                                count += 1
                                learning_times[location] = count
                                with open(file_path, "w", encoding="utf-8") as f:
                                    for loc_name, cnt in learning_times.items():
                                        f.write(f"{loc_name}: Learning Times: {cnt}\n")
                                my_select.set_target("Loading Start")
                                my_select.start_new_episode()
                                start_back_time = robot.getTime()
                                command = "Go back"
                                continue

                        case "Go back":

                            distance = judge_distance(Start_location[0], Start_location[1])
                            motor_controller.Stop()
                            my_select.Run_Learning_Q_Learning(v)

                            if protect == 1:
                                if fail_times >= 30:
                                    print(f"Fail too many times, restart\n")
                                    fail_times = 0
                                    reset_robot_pose_to_start(start_pos, start_yaw)
                                    break
                                print("Learning fail, back to last state and learn again\n")
                                reset_robot_pose_to_start(robot_last_location, robot_last_yaw)
                                fail_fps = 0
                                stuck_fps = 0
                                fail_times += 1
                                protect = 0
                                my_select.choice -= 1
                                my_select.state = 0
                                my_select.at_cross = 0
                                now = robot.getTime()
                                my_select._next_cross_check_time = now
                                my_select._last_pattern = 0
                                my_select._pattern_count = 0
                                my_select._last_cross_time = 0.0
                                my_select.choice_error = 1
                                continue

                            if distance <= 0.9:
                                my_tracker.Track_Way(v)
                            if distance <= my_home_radius:
                                print(f"Arrive the Start Station\n")
                                order = 0
                                rotate_to_head(start_yaw, v)

                                end_back_time = robot.getTime()

                                # The time back to start station
                                total_time_back = end_back_time - start_back_time

                                choice_back = my_select.Report_Choice()
                                send_complete_flag = 0
                                back_complete = 1
                                error = fail_times

                                back_score = score(total_time_back, choice_back,
                                                   send_complete_flag, back_complete, error)

                                fail_times = 0
                                my_select.Q_End_Episode(back_score)
                                if count >= need_learning_times:
                                    print(f"{location} Learning Times Max {need_learning_times}，Switch Next Target\n")
                                    break
                                command = "Waiting"
                                continue

                        case "Waiting":
                            motor_controller.Stop()
                            my_select.cross_lock = False
                            print(order)
                            if order == 0:
                                motor_controller.Stop()
                                send_complete = 0
                                back_complete = 0
                                order += 1
                                my_select.start_new_episode()
                                my_select.choice = 0
                                command = "Sending"
                                my_select.set_target(location)
                                my_select.start_new_episode()
                                print(f"location: {location}\n")
                                print(f"destination: {destination}\n")
                                continue

        if key == ord("4"):
            print(f"The Q_Learning Deploy mode has been selected\n")

            # The location of Start Station
            Start_location = my_destination_map['Loading Start']

            my_select.set_target("Loading Start")

            while robot.step(timestep) != -1:

                match command:
                    case "Sending":
                        v, massage_time = compute_speed_with_traffic(base_speed, my_traffic_light, camera, my_select, massage_time)
                        # Whether leave the start station
                        beginning = judge_distance(Start_location[0], Start_location[1])
                        if beginning <= 1:
                            my_tracker.Track_Way(v)
                            continue

                        # Feedback of Sending
                        if feedback_times == 0:
                            feedback_data = generate_feedback(robot.getTime(), destination, command)
                            send_feedback(feedback_data)
                            feedback_times = 1

                        # Using Q Learning Deploy
                        my_select.Run_Deploy_Q_Learning(v)

                        # The distance of target
                        distance = judge_distance(location[0], location[1])

                        if distance <= my_goal_radius:
                            if feedback_times == 1:
                                feedback_data = generate_feedback(robot.getTime(), destination, "Complete")
                                send_feedback(feedback_data)
                                feedback_times = 0
                            order -= 1
                            print(f"Order Completed\n")
                            location = None
                            command = "Go back"
                            my_select.set_target("Loading Start")
                            continue

                    case "Go back":
                        v, massage_time = compute_speed_with_traffic(base_speed, my_traffic_light, camera, my_select, massage_time)
                        distance = judge_distance(Start_location[0], Start_location[1])
                        motor_controller.Stop()
                        if distance <= 1:
                            my_tracker.Track_Way(v)
                            if distance <= my_home_radius:
                                print(f"Arrive the station\n")
                                feedback_data = generate_feedback(robot.getTime(), destination, "Home")
                                send_feedback(feedback_data)
                                feedback_times = 0
                                rotate_to_head(start_yaw, v)
                                command = "Waiting"
                                continue
                            continue
                        my_select.Run_Deploy_Q_Learning(v)

                    case "Waiting":
                        motor_controller.Stop()
                        my_select.choice = 0
                        print(f"Waiting for new order\n")
                        print(order)

                        if order == 0:
                            # wait for new order
                            motor_controller.Stop()
                            order_result = check_receiver()
                            send_complete = 0
                            back_complete = 0

                            if order_result is not None:
                                location, destination = order_result
                                order += 1
                                start_sending_time = robot.getTime()
                                command = "Sending"

                                print(f"location: {location}\n")
                                print(f"destination: {destination}\n")
                                my_select.set_target(destination)
                                continue

                        if location is None:
                            location = check_receiver()
                            motor_controller.Stop()
                            if location is None:
                                continue

        if key == ord("5"):
            print(f"测试camera\n")
            while robot.step(timestep) != -1:
                camera.Run_Traffic_Light_Camera()



