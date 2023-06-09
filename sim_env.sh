#!/bin/bash
sudo unzip ./models.zip -d ~/.gazebo
sudo cp -r ./apriltag ~/.gazebo/models/apriltag
sudo cp -r ./files ~/YOUR_WORKSPACE_PATH
sudo cp -r ./start_simulation.sh ~/YOUR_WORKSPACE_PATH


