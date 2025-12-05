import json

import my_timedelay as M_D
import my_track as M_T
import motor_controller as M_C
import random
import re
from collections import defaultdict
import math
import os
import ast
import my_led as M_L
import my_camera as M_CA

class MySelection:
    def __init__(self, robot, keyboard):
        self.robot = robot
        self.at_cross = 0
        self.state = 0
        self.select = 0
        self.led = M_L.LedController(self.robot)
        self.delay = M_D.Delaytime(self.robot)
        self.tracker = M_T.Tracker(self.robot)
        self.camera = M_CA.MyCamera(self.robot, keyboard)
        self.motor_controller = M_C.MotorController(self.robot)
        self.timestep = 0
        self.key = keyboard
        self.tracker_left_middle = None
        self.tracker_right_middle = None
        self.tracker_front_middle = None
        self.tracker_left_middle_value = 0
        self.tracker_right_middle_value = 0
        self.tracker_front_middle_value = 0
        self.v = 0
        self.station = 0
        self.choice = 0
        self.confirm = 0
        self.decisions = None
        self.last_pos = None
        self.last_move_time = 0.0
        self.stuck_dist_eps = 0.05
        self.stuck_time_limit = 4.0
        self.my_gps = robot.getDevice("robot_gps")
        self.robot_last_location = None
        self.robot_last_yaw = None
        self.my_imu = robot.getDevice("inertial unit")
        self.choice_error = 0
        self.last_action_per_state = {}
        self.punish_value = 1.0
        self.traffic_judge = 0
        self.traffic_done = 0

        # Action space
        self.actions_all = ['TurnLeft', 'TurnRight', 'GoStraight']
        # Q table: state -> { action: value }
        self.Q = defaultdict(
            lambda: defaultdict(
                lambda: {a: 0.0 for a in self.actions_all}
            )
        )
        # Learning rate/Discount/Exploration rate
        self.alpha = 0.1
        self.gamma = 0.95
        self.epsilon = 0.2
        self.episode_transitions = []
        self.current_target = "Unknown"

    # Init all the modules
    def Init_ALL_Modules(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.tracker_left_middle = self.robot.getDevice("Distance_Sensor_Left_Middle")
        self.tracker_right_middle = self.robot.getDevice("Distance_Sensor_Right_Middle")
        self.tracker_front_middle = self.robot.getDevice("Distance_Sensor_Front_Middle")

        self.tracker.Init_ALL_Distance_Sensor_And_Motors()
        self.tracker.Enable_Distance_sensor(self.timestep)
        self.tracker_left_middle.enable(self.timestep)
        self.tracker_right_middle.enable(self.timestep)
        self.tracker_front_middle.enable(self.timestep)
        self.my_imu.enable(self.timestep)

        self.motor_controller.Init_All_Motors()
        self.motor_controller.Stop()
        self.at_cross = 0
        self.state = 0
        self.select = 0
        self.route_done = False
        self.my_gps.enable(self.timestep)

        self.load_Q()

        self.led.InitLed()

        self.camera.Init_All_Module()

    def _discretize_pos(self, x, y, resolution=0.2):
        xb = round(x / resolution)
        yb = round(y / resolution)
        return xb, yb

    def _get_state(self):
        cross_type = int(self.at_cross)

        x, y, z = self.my_gps.getValues()
        xb, yb = self._discretize_pos(x, y)
        return (cross_type, xb, yb)

    def _valid_actions(self, cross_type: int):
        if cross_type == 1:      # T-junction: Left / Right
            return ['TurnLeft', 'TurnRight']
        elif cross_type == 2:    # Cross road: Left / Right / Straight
            return ['TurnLeft', 'TurnRight', 'GoStraight']
        elif cross_type == 3:    # |- junction: Right / Straight
            return ['TurnRight', 'GoStraight']
        elif cross_type == 4:    # -| junction: Left / Straight
            return ['TurnLeft', 'GoStraight']
        else:
            return []

    def _epsilon_greedy_action(self, state, valid_actions):
        if not valid_actions:
            return None

        # Explore
        if random.random() < self.epsilon:
            return random.choice(valid_actions)

        # Utilization: Select the action with the largest Q value
        q_s = self.Q[self.current_target][state]
        best_a = max(valid_actions, key=lambda a: q_s.get(a, 0.0))
        return best_a

    def _greedy_action(self, state, valid_actions):
        if not valid_actions:
            return None
        q_s = self.Q[self.current_target][state]
        best_a = max(valid_actions, key=lambda a: q_s.get(a, 0.0))
        return best_a

    def _exec_action(self, cross_type, action_name):
        if action_name is None:
            return

        sel = 0
        if cross_type == 1:  # T 字路口：左 / 右
            self.robot_last_location = self.get_robot_location()
            self.robot_last_yaw = self.get_yaw()
            if action_name == 'TurnLeft':
                sel = -10
            elif action_name == 'TurnRight':
                sel = 10

        elif cross_type == 2:  # 十字路口：左 / 右 / 直行
            self.robot_last_location = self.get_robot_location()
            self.robot_last_yaw = self.get_yaw()
            if action_name == 'TurnLeft':
                sel = -5
            elif action_name == 'TurnRight':
                sel = 5
            elif action_name == 'GoStraight':
                sel = 0

        elif cross_type == 3:  # |-
            self.robot_last_location = self.get_robot_location()
            self.robot_last_yaw = self.get_yaw()
            if action_name == 'TurnRight':
                sel = 10
            elif action_name == 'GoStraight':
                sel = 0

        elif cross_type == 4:  # -|
            self.robot_last_location = self.get_robot_location()
            self.robot_last_yaw = self.get_yaw()
            if action_name == 'TurnLeft':
                sel = -10
            elif action_name == 'GoStraight':
                sel = 0

        self.state = 1
        _ = self.Selection(self.state, sel, self.v)

        self.state = self.Selection(self.state, 0, self.v)

    # Create a new episode
    def start_new_episode(self):
        self.episode_transitions = []
        self.choice = 0

    # Figure out the Q score
    def Q_End_Episode(self, total_score: float):
        if not self.episode_transitions:
            return

        r_terminal = total_score / 100.0
        T = len(self.episode_transitions)

        q_dest = self.Q[self.current_target]

        for t in range(T):
            s_t, a_t = self.episode_transitions[t]
            if t == T - 1:
                r = r_terminal
                max_next = 0.0
            else:
                r = 0.0
                s_next, _ = self.episode_transitions[t + 1]
                max_next = max(q_dest[s_next].values())

            old_q = q_dest[s_t][a_t]
            target = r + self.gamma * max_next
            q_dest[s_t][a_t] = old_q + self.alpha * (target - old_q)

        self.epsilon = max(0.05, self.epsilon * 0.99)
        print(f"[Q_Learning] current epsilon = {self.epsilon:.3f}")
        self.save_Q()
        self.episode_transitions = []

    # Save the Q table
    def save_Q(self, path = "q_table.json"):
        data = {}

        for dest_name, table in self.Q.items():
            inner_table = {}
            for s, adict in table.items():

                cross_type = s[0]
                allowed = set(self._valid_actions(cross_type))

                clean_adict = {a: v for a, v in adict.items() if a in allowed}

                inner_table[str(s)] = clean_adict
            data[dest_name] = inner_table

        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        print("[Q_Learning] All destinations have been saved Q：", list(data.keys()))

    # Load the Q table
    def load_Q(self, path = "q_table.json"):
        if not os.path.exists(path):
            return
        with open(path, "r", encoding = "utf-8") as f:
            data = json.load(f)

        self.Q = defaultdict(
            lambda: defaultdict(
                lambda: {a: 0.0 for a in self.actions_all}
            )
        )

        for target_name, table in data.items():
            for s_str, adict in table.items():
                try:
                    s = ast.literal_eval(s_str)  # (cross_type, xb, yb)
                except Exception:
                    continue

                cross_type = s[0]
                allowed = set(self._valid_actions(cross_type))

                for a, v in adict.items():
                    if a in allowed:
                        self.Q[target_name][s][a] = float(v)

    # Set the target
    def set_target(self, target_name: str):
        print(f"The current destination is switched to: {target_name}\n")
        self.current_target = target_name

    # The decision sort to find the best decision
    def Decision_Sort(self):
        file_path = "learning_decision.txt"

        self.decisions = defaultdict(list)

        with open(file_path, 'r', encoding = 'utf-8') as f:
            for line in f:
                match = re.search(r"(.*?)End Score:\s*([\d\.]+)", line)
                if match:
                    decision_text = match.group(1).strip()
                    score = float(match.group(2))

                    path_match = re.search(r"Decision:\s*(.*)", decision_text)
                    if path_match:
                        actions_raw = path_match.group(1)
                        actions = [a.strip() for a in actions_raw.split("->") if a.strip()]
                    else:
                        actions = []

                    location_match = re.search(r"(.*?)\s*Decision:", decision_text)
                    location = location_match.group(1).strip() if location_match else "Unknow location"
                    self.decisions[location].append({"decision_text" : decision_text,
                                      "actions" : actions,
                                      "score" : score,
                                      "location" : location})

        for loc in self.decisions:
            self.decisions[loc].sort(key=lambda x: x["score"], reverse=True)

    # Get the robot location
    def get_robot_location(self):
        x, y, z = self.my_gps.getValues()
        return (x, y)

    # Get the robot yaw
    def get_yaw(self):
        if self.my_imu is not None:
            roll, pitch, yaw = self.my_imu.getRollPitchYaw()
            return yaw
        else:
            return None

    # Save the best decision into the file
    def Save_Decision_Data(self):
        with open('deploy_decision.txt', 'w', encoding='utf-8') as f:
            f.write("")
        for loc in sorted(self.decisions.keys()):
            elements = self.decisions[loc]
            best_decision = elements[0]
            with open('deploy_decision.txt', 'a', encoding='utf-8') as f:
                f.write(f"Location: {loc}\nBest decision:")
                f.write(f"{best_decision['actions']}\n")

    # Running the decision save
    def Deploy_Decision_Save(self):
        self.Decision_Sort()
        self.Save_Decision_Data()

    # Report the number of choice
    def Report_Choice(self):
        return self.choice

    # The Q_Learning fail execution
    def Learning_Fail_Q_Learning(self):
        print("Misjudgment\n")
        self.Q_End_Episode(-100.0)

    # The assessment learning fail execution
    def Learning_Fail_Score_Learning(self):
        print("Misjudgment\n")
        with open("learning_decision.txt", 'r', encoding='utf-8') as f:
            lines = f.readlines()

        lines = [line for line in lines if line.strip()]

        if lines:
            lines = lines[:-1]

        with open("learning_decision.txt", 'w', encoding='utf-8') as f:
            f.writelines(lines)

    # Check whether the robot is stuck
    def check_stuck(self):
        if self.last_pos is None:
            self.last_pos = self.get_robot_location()
            self.last_move_time = self.robot.getTime()
            return False

        current_pos = self.get_robot_location()
        dx = current_pos[0] - self.last_pos[0]
        dy = current_pos[1] - self.last_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)
        now_t = self.robot.getTime()

        if dist > self.stuck_dist_eps:
            self.last_pos = current_pos
            self.last_move_time = now_t
            return False

        if now_t - self.last_move_time > self.stuck_time_limit:
            print("If the robot remains in the same position for more than 4 seconds\n")
            print("It is judged as a judgment error\n")
            return True

        return False

    # Reset the robot stuck state
    def Reset_Stuck_State(self):
        self.last_pos = None
        self.last_move_time = self.robot.getTime()

    # The Actions Selection
    def Selection(self, state, selection, v):

        self.v = v
        # Choose to turn right
        if selection == 10:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Right(v)
                self.led.TurnRightLight()
                turn_time = 10 * 0.8 / v
                self.delay.Delay(turn_time)
                selection = 0
                return selection

        # Choose to turn left
        if selection == -10:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Left(v)
                self.led.TurnLeftLight()
                turn_time = 10 * 0.8 / v
                self.delay.Delay(turn_time)
                selection = 0
                return selection

        # Choose to turn right at the crossroads
        if selection == 5:
            while self.robot.step(self.timestep) != -1:
                self.motor_controller.Turn_Right(v)
                self.led.TurnRightLight()
                turn_time = 10 * 0.8 / v
                self.delay.Delay(turn_time)
                selection = 0
                return selection

        # Choose to turn left at the crossroads
        if selection == -5:
            while self.robot.step(self.timestep) != -1:
                self.motor_controller.Turn_Left(v)
                self.led.TurnLeftLight()
                turn_time = 10 * 0.8 / v
                self.delay.Delay(turn_time)
                selection = 0
                return selection

        # Choose to go straight
        if selection == 0:
            start_time = self.robot.getTime()
            go_straight_time = 10 / v
            while self.robot.step(self.timestep) != -1:
                self.tracker.Track_Way(self.v)
                current_time = self.robot.getTime()

                if current_time - start_time >= go_straight_time:
                    state = 0
                    return state

    def give_robot_last_location(self):
        return self.robot_last_location

    def give_robot_last_yaw(self):
        return self.robot_last_yaw

    def detect_traffic_red(self):
        traffic_light = self.camera.Run_Traffic_Light_Camera()
        if traffic_light == "RED":
            print(f"RED Light in injunction, STOP!\n")
            self.v = 0
            self.traffic_judge = 1
        else:
            self.v = 10
            self.traffic_judge = 0

    # The Learning Q_Learning method mode
    def Run_Learning_Q_Learning(self, v):

        self.tracker_left_middle_value = self.tracker_left_middle.getValue()
        self.tracker_right_middle_value = self.tracker_right_middle.getValue()
        self.tracker_front_middle_value = self.tracker_front_middle.getValue()
        self.v = v

        detected_cross = 0
        now = self.robot.getTime()
        if not hasattr(self, "_next_cross_check_time"):
            self._next_cross_check_time = 0.0

        # Only turn right
        if self.tracker_right_middle_value >= 700 \
                and self.tracker_left_middle_value < 500 \
                and self.tracker_front_middle_value < 500:
            self.state = 1
            self.station += 1

        # Only turn left
        elif self.tracker_right_middle_value < 500 \
                and self.tracker_left_middle_value >= 700 \
                and self.tracker_front_middle_value < 500:
            self.station -= 1
            self.state = 1
        else:
            self.station = 0

        if self.state == 0:
            if now >= self._next_cross_check_time:
                f = self.tracker_front_middle_value
                l = self.tracker_left_middle_value
                r = self.tracker_right_middle_value

                HIGH = 700
                MID = 500

                front_on = f >= HIGH
                left_on = l >= HIGH
                right_on = r >= HIGH

                pattern = 0

                if left_on and right_on and front_on:
                    pattern = 2  # cross type

                elif left_on and right_on and not front_on and f < MID:
                    pattern = 1  # T type

                elif right_on and front_on and not left_on and l < MID:
                    pattern = 3  # |- type

                elif left_on and front_on and not right_on and r < MID:
                    pattern = 4  # -| type

                if not hasattr(self, "_last_pattern"):
                    self._last_pattern = 0
                    self._pattern_count = 0

                if pattern != self._last_pattern:
                    self._last_pattern = pattern
                    self._pattern_count = 1
                else:
                    if pattern != 0:
                        self._pattern_count += 1

                detected_cross = 0
                if self._pattern_count >= 3 and pattern != 0 and self.state != 1:
                    detected_cross = pattern
                    self._next_cross_check_time = now + 2.5

        now = self.robot.getTime()
        if not hasattr(self, "_last_cross_time"):
            self._last_cross_time = 0.0
            self._cross_cooldown = 1

        if detected_cross != 0:
            if now - self._last_cross_time < self._cross_cooldown:
                self.at_cross = 0
            else:
                self.at_cross = detected_cross
                self._last_cross_time = now
                print(f"[Learning-Q] 确认新路口类型 {self.at_cross}，时间={now:.3f}")
        else:
            self.at_cross = 0

        # ========== 关键：在路口用 Q-learning 选动作 ==========
        if self.at_cross in (1, 2, 3, 4):
            state = self._get_state()
            valid_actions = self._valid_actions(self.at_cross)
            action = self._epsilon_greedy_action(state, valid_actions)

            print(f"[Learning-Q] 路口类型={self.at_cross}，状态={state}，选动作={action}")

            # 记录到本局轨迹里
            self.episode_transitions.append((state, action))
            # 执行动作
            self.motor_controller.Stop()
            self._exec_action(self.at_cross, action)

            self.at_cross = 0
        else:
            self.tracker.Track_Way(self.v)

        if self.station >= 5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Right(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station <= -5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Left(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station == 0 and self.at_cross == 0:
            self.tracker.Track_Way(self.v)

    # The Deploy Q_Learning method mode
    def Run_Deploy_Q_Learning(self, v):

        self.tracker_left_middle_value = self.tracker_left_middle.getValue()
        self.tracker_right_middle_value = self.tracker_right_middle.getValue()
        self.tracker_front_middle_value = self.tracker_front_middle.getValue()
        self.v = v

        detected_cross = 0
        now = self.robot.getTime()
        if not hasattr(self, "_next_cross_check_time"):
            self._next_cross_check_time = 0.0

        if (self.tracker_right_middle_value >= 500
                and self.tracker_left_middle_value < 500
                and self.tracker_front_middle_value < 500):
            self.state = 1
            self.station += 1

        elif (self.tracker_right_middle_value < 500
              and self.tracker_left_middle_value >= 500
              and self.tracker_front_middle_value < 500):
            self.station -= 1
            self.state = 1
        else:
            self.station = 0

        if self.station == 0:
            if now >= self._next_cross_check_time:
                f = self.tracker_front_middle_value
                l = self.tracker_left_middle_value
                r = self.tracker_right_middle_value

                HIGH = 700
                MID = 500

                front_on = f >= HIGH
                left_on = l >= HIGH
                right_on = r >= HIGH

                pattern = 0
                if left_on and right_on and front_on:
                    pattern = 2  # cross type
                elif left_on and right_on and not front_on and f < MID:
                    pattern = 1  # T type
                elif right_on and front_on and not left_on and l < MID:
                    pattern = 3  # |- type
                elif left_on and front_on and not right_on and r < MID:
                    pattern = 4  # -| type

                if not hasattr(self, "_last_pattern"):
                    self._last_pattern = 0
                    self._pattern_count = 0

                if pattern != self._last_pattern:
                    self._last_pattern = pattern
                    self._pattern_count = 1
                else:
                    if pattern != 0:
                        self._pattern_count += 1

                detected_cross = 0
                if self._pattern_count >= 3 and pattern != 0 and self.state != 1:
                    detected_cross = pattern
                    self._next_cross_check_time = now + 2.5

        # 路口冷却
        now = self.robot.getTime()
        if not hasattr(self, "_last_cross_time"):
            self._last_cross_time = 0.0
            self._cross_cooldown = 1.0

        if detected_cross != 0:
            if now - self._last_cross_time < self._cross_cooldown:
                self.at_cross = 0
            else:
                self.at_cross = detected_cross
                self._last_cross_time = now
                print(f"[Deploy-Q] confirm the new type of intersection {self.at_cross}, Time = {now:.3f}")
        else:
            self.at_cross = 0

        if self.at_cross in (1, 2, 3, 4):
            # current status: (cross_type, xb, yb)
            state = self._get_state()
            # find the actions depend on the type of the junction
            valid_actions = self._valid_actions(self.at_cross)

            if not valid_actions:
                print(f"[Deploy-Q] Status = {state} No legal action, maintain line patrol\n")
            else:
                # get the Q value dictionary of current status
                q_s = self.Q[self.current_target][state]

                # choose the Max value in current actions
                # find the max action
                q_s = self.Q[self.current_target][state]
                max_q_value = max(q_s.get(a, 0.0) for a in valid_actions)

                # if there are parallel largest actions, they will be randomly selected
                best_actions = [a for a in valid_actions if q_s.get(a, 0.0) == max_q_value]
                action = random.choice(best_actions)

                print(f"[Deploy-Q] Intersection type = {self.at_cross}, status = {state},"
                      f"Q = { {a: q_s.get(a, 0.0) for a in valid_actions} }，"
                      f"execute the action = {action}\n")

                # execute the actions
                self.motor_controller.Stop()
                self._exec_action(self.at_cross, action)

            self.at_cross = 0

        if self.station >= 5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Right(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station <= -5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Left(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        # Track the way
        if self.station == 0 and self.at_cross == 0:
            self.tracker.Track_Way(self.v)

    # The Learning Assessment method mode
    def Run_Score_Learning(self, v):

        self.tracker_left_middle_value = self.tracker_left_middle.getValue()
        self.tracker_right_middle_value = self.tracker_right_middle.getValue()
        self.tracker_front_middle_value = self.tracker_front_middle.getValue()
        self.v = v

        detected_cross = 0
        now = self.robot.getTime()
        if not hasattr(self, "_next_cross_check_time"):
            self._next_cross_check_time = 0.0

        # The situation where only a right turn is allowed
        if self.tracker_right_middle_value >= 500 \
                and self.tracker_left_middle_value < 500 \
                and self.tracker_front_middle_value < 500:
            self.state = 1
            self.station += 1

        # The situation where only a left turn is allowed
        elif self.tracker_right_middle_value < 500 \
                and self.tracker_left_middle_value >= 500 \
                and self.tracker_front_middle_value < 500:
            self.station -= 1
            self.state = 1
        else:
            self.station = 0

        if self.state == 0:
            if now < self._next_cross_check_time:
                pass
            else:
                f = self.tracker_front_middle_value
                l = self.tracker_left_middle_value
                r = self.tracker_right_middle_value

                HIGH = 700
                MID = 500

                front_on = f >= HIGH
                left_on = l >= HIGH
                right_on = r >= HIGH

                pattern = 0

                if left_on and right_on and front_on:
                    pattern = 2  # cross type

                elif left_on and right_on and not front_on and f < MID:
                    pattern = 1  # T type

                elif right_on and front_on and not left_on and l < MID:
                    pattern = 3  # |- type

                elif left_on and front_on and not right_on and r < MID:
                    pattern = 4  # -| type

                if not hasattr(self, "_last_pattern"):
                    self._last_pattern = 0
                    self._pattern_count = 0

                if pattern != self._last_pattern:
                    self._last_pattern = pattern
                    self._pattern_count = 1
                else:
                    if pattern != 0:
                        self._pattern_count += 1

                detected_cross = 0
                if self._pattern_count >= 3 and pattern != 0 and self.state != 1:
                    detected_cross = pattern
                    self._next_cross_check_time = now + 2.5

        now = self.robot.getTime()
        if not hasattr(self, "_last_cross_time"):
            self._last_cross_time = 0.0
            self._cross_cooldown = 1

        if detected_cross != 0:
            if now - self._last_cross_time < self._cross_cooldown:
                self.at_cross = 0
            else:
                self.at_cross = detected_cross
                self._last_cross_time = now
                print(f"[Score_Learning] confirm the new intersection type: {self.at_cross}, time ={now:.3f}")
        else:
            self.at_cross = 0

        # Match road conditions
        match self.at_cross:
            case 1:
                self.choice += 1
                choice = random.choice(['left', 'right'])
                self.motor_controller.Stop()
                if choice == 'left':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnLeft->')
                    self.select = -10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn left\n")
                    self.state = self.Selection(self.state, 0, self.v)

                elif choice == 'right':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnRight->')
                    self.select = 10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Right\n")
                    self.state = self.Selection(self.state, 0, self.v)
                self.at_cross = 0

            case 2:
                self.choice += 1
                choice = random.choice(['left', 'right', 'straight'])
                self.motor_controller.Stop()
                if choice == 'left':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnLeft->')
                    self.select = -5
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn left\n")
                    self.state = self.Selection(self.state, 0, self.v)

                elif choice == 'right':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnRight->')
                    self.select = 5
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn right\n")
                    self.state = self.Selection(self.state, 0, self.v)

                elif choice == 'straight':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('GoStraight->')
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go straight\n")
                self.at_cross = 0

            case 3:
                self.choice += 1
                choice = random.choice(['right', 'straight'])
                self.motor_controller.Stop()
                if choice == 'right':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnRight->')
                    self.select = 10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn right\n")
                    self.state = self.Selection(self.state, 0, self.v)

                elif choice == 'straight':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('GoStraight->')
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go straight\n")
                self.at_cross = 0

            case 4:
                self.choice += 1
                choice = random.choice(['left', 'straight'])
                self.motor_controller.Stop()
                if choice == 'left':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('TurnLeft->')
                    self.select = -10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn left\n")
                    self.state = self.Selection(self.state, 0, self.v)

                elif choice == 'straight':
                    with open('learning_decision.txt', 'a', encoding='utf-8') as f:
                        f.write('GoStraight->')
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go straight\n")
                self.at_cross = 0

        if self.station >= 5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Right(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station <= -5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Left(self.v)
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        # Go with the road
        if self.station == 0:
            self.tracker.Track_Way(self.v)

    # The Deploy Assessment method mode
    def Run_Deploy_Score_Learning(self, v, decisions):
        if not decisions:
            self.tracker.Track_Way(v)
            return

        if self.choice >= len(decisions):
            self.at_cross = 0
            self.station = 0
            self.state = 0
            self.tracker.Track_Way(v)
            return

        self.tracker_left_middle_value = self.tracker_left_middle.getValue()
        self.tracker_right_middle_value = self.tracker_right_middle.getValue()
        self.tracker_front_middle_value = self.tracker_front_middle.getValue()
        self.v = v

        detected_cross = 0
        now = self.robot.getTime()
        if not hasattr(self, "_next_cross_check_time"):
            self._next_cross_check_time = 0.0

        # The situation where only right turns are allowed
        if self.tracker_right_middle_value >= 700 \
                and self.tracker_left_middle_value < 500 \
                and self.tracker_front_middle_value < 500:
            self.state = 1
            self.station += 1

        # The situation where only a left turn is allowed
        elif self.tracker_right_middle_value < 500 \
                and self.tracker_left_middle_value >= 700 \
                and self.tracker_front_middle_value < 500:
            self.station -= 1
            self.state = 1
        else:
            self.station = 0

        if self.station == 0:
            if now < self._next_cross_check_time:
                pass
            else:
                f = self.tracker_front_middle_value
                l = self.tracker_left_middle_value
                r = self.tracker_right_middle_value

                HIGH = 700
                MID = 500

                front_on = f >= HIGH
                left_on = l >= HIGH
                right_on = r >= HIGH

                # By default, it is assumed that there are no intersections
                pattern = 0

                if left_on and right_on and front_on:
                    pattern = 2  # cross type
                elif left_on and right_on and not front_on and f < MID:
                    pattern = 1  # T type

                elif right_on and front_on and not left_on and l < MID:
                    pattern = 3  # |- type

                elif left_on and front_on and not right_on and r < MID:
                    pattern = 4  # -| type

                if not hasattr(self, "_last_pattern"):
                    self._last_pattern = 0
                    self._pattern_count = 0

                if pattern != self._last_pattern:
                    self._last_pattern = pattern
                    self._pattern_count = 1
                else:
                    if pattern != 0:
                        self._pattern_count += 1

                detected_cross = 0
                if self._pattern_count >= 3 and pattern != 0 and self.state != 1:
                    detected_cross = pattern
                    self._next_cross_check_time = now + 2.5

        now = self.robot.getTime()
        if not hasattr(self, "_last_cross_time"):
            self._last_cross_time = 0.0
            self._cross_cooldown = 1

        if detected_cross != 0:
            if now - self._last_cross_time < self._cross_cooldown:
                self.at_cross = 0
            else:
                self.at_cross = detected_cross
                self._last_cross_time = now
                print(f"[Deploy] confirm the new intersection type {self.at_cross}, time ={now:.3f}")
        else:
            self.at_cross = 0

        match self.at_cross:
            case 1:
                idx = self.choice
                choice = decisions[idx]
                self.choice += 1

                print(f"[Deploy-S] Intersection type = 1(T type), execute the {idx + 1}/{len(decisions)} decision: {choice}\n")

                self.motor_controller.Stop()
                if choice == 'TurnLeft':
                    self.traffic_done = 1
                    self.select = -10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Left\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                if choice == 'TurnRight':
                    self.traffic_done = 1
                    self.select = 10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Right\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                self.at_cross = 0

            case 2:

                idx = self.choice
                choice = decisions[idx]
                self.choice += 1

                print(f"[Deploy-S] Intersection type = 2(cross type)，execute the {idx + 1}/{len(decisions)} decision: {choice}\n")

                self.motor_controller.Stop()
                if choice == 'TurnLeft':
                    self.traffic_done = 1
                    self.select = -5
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Left\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                if choice == 'TurnRight':
                    self.traffic_done = 1
                    self.select = 5
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Right\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                if choice == 'GoStraight':
                    self.traffic_done = 1
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go Straight\n")
                    self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                    self.traffic_done = 0
                self.at_cross = 0

            case 3:
                idx = self.choice
                choice = decisions[idx]
                self.choice += 1

                print(f"[Deploy-S] Intersection type = 3(|- type)，execute the {idx + 1}/{len(decisions)} decision: {choice}\n")

                self.motor_controller.Stop()
                if choice == 'TurnRight':
                    self.traffic_done = 1
                    self.select = 10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Right\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                if choice == 'GoStraight':
                    self.traffic_done = 1
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go Straight\n")
                    self.traffic_done = 0
                self.at_cross = 0

            case 4:
                idx = self.choice
                choice = decisions[idx]
                self.choice += 1

                print(f"[Deploy-S] Intersection type = 4(-| type)，execute the {idx + 1}/{len(decisions)} decision: {choice}\n")

                self.motor_controller.Stop()
                if choice == 'TurnLeft':
                    self.traffic_done = 1
                    self.select = -10
                    self.state = 1
                    self.select = self.Selection(self.state, self.select, self.v)
                    print("Select: Turn Left\n")
                    self.state = self.Selection(self.state, 0, self.v)
                    self.traffic_done = 0
                if choice == 'GoStraight':
                    self.traffic_done = 1
                    self.select = 0
                    self.state = 1
                    self.state = self.Selection(self.state, self.select, self.v)
                    print("Select: Go Straight\n")
                    self.traffic_done = 0
                self.at_cross = 0

        if self.station >= 5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Right(self.v)
                self.led.TurnRightLight()
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station <= -5:
            while self.robot.step(self.timestep) != -1:
                self.tracker_front_middle_value = self.tracker_front_middle.getValue()
                self.motor_controller.Turn_Left(self.v)
                self.led.TurnLeftLight()
                self.delay.Delay(0.5)
                if self.tracker_front_middle_value >= 500:
                    self.station = 0
                    self.state = 0
                    start_time = self.robot.getTime()
                    while self.robot.step(self.timestep) != -1:
                        current_time = self.robot.getTime()
                        self.tracker.Track_Way(self.v)
                        if current_time - start_time >= 1:
                            self.motor_controller.Stop()
                            break
                    break

        if self.station == 0:
            self.tracker.Track_Way(self.v)

