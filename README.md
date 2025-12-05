# Intelligent_Robotics_Project_Group4
This repository contains a Webots-based autonomous driving project using Q-Learning and evaluation learning reinforcement learning and a custom-designed set of controller modules that handle perception, decision-making, and robot actuation.
All reinforcement learning logic, robot behaviour control, and environment configuration were implemented by us unless otherwise specified.

Tips: The uploaded code has already been learned. If you want to relearn it, you need to clear learning_decision-.txt
#
# 1. Project Structure
    my_project_group4/
      Maps/
        map3.png
      controllers/
        my_controller/
            my_controller.py
            motor_controller.py
            my_camera.py
            my_led.py
            my_selection.py
            my_selection_q_learning.py
            my_timedelay.py
            my_track.py
            offline_q_learning.py
            build_deploy_from_q.py
            learning_decision.txt
            deploy_decision.txt
            Q_Learning_times.txt
            q_table.json
            __pycache__/ (auto-generated)

        my_house1_buy_syetem/
            my_house1_buy_syetem.py

        my_traffic_light_controller/
            my_traffic_light_controller.py

      worlds/
          My_Track_robot_work1.wbt
          .My_Track_robot_work1.wbproj
          .My_Track_robot_work1.jpg
#
# 2. Features Implemented by Ourselves
1) Reinforcement Learning System
   
     Implemented modules:
   
        offline_q_learning.py
   
        my_selection_q_learning.py
   
        q_table.json
   
        Decision logging (*.txt)
   
    Includes:
   
      Q-table training
   
      Reward shaping
   
      Action/state design

      Policy deployment

2) Robot Control & Perception (Fully Written by Us)

        my_controller.py – main controller
    
        motor_controller.py – motor abstraction
        
        my_camera.py – camera frame processing
        
        my_track.py – lane/track detection logic
        
        my_led.py – LED indicator logic
        
        my_timedelay.py – time control
        
        my_selection.py – deterministic action selection
   
3) Custom Webots World Design

        My_Track_robot_work1.wbt
        
        Custom map assets under Maps/
#
# 3. Pre-Programmed Packages Used (Not Implemented by Us)

Webots Built-in Features

    Motor API
    
    Sensor API
    
    Simulation step management
    
    World editor features

Python Standard Libraries

    json, time, random, os, math
#
# 4. How to Run the Project
1) Open Webots

2) Load world:
   
        worlds/My_Track_robot_work1.wbt

3) Set controller to:

        controllers/my_controller/my_controller.py

4) Run simulation → Robot drives autonomously

#
# 5. Acknowledgements
This project uses the Webots simulation platform and standard Python libraries.

All core algorithms and robot control logic were implemented by the project group.
#



