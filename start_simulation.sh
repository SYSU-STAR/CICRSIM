#!/bin/bash

gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
sleep 5s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation env_simulation.launch;exec bash;"
sleep 5s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation uav_simulation.launch;exec bash;"
sleep 3s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation referee_system.launch;exec bash;"
sleep 2s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;rosrun uav_simulation command_process.py;exec bash;"
sleep 2s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;rosrun uav_simulation keyboard_control.py;exec bash;"
sleep 2s



