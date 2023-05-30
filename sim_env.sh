#!/bin/bash
sudo cp -r ./apriltag ~/.gazebo/models
sudo unzip ./models.zip -d ~/.gazebo/models
sudo cp -r ./files ~/catkin_ws
sudo cp -r ./start_simulation.sh ~/catkin_ws


