#!/bin/env bash
source /opt/ros/noetic/setup.bash           # Source ros setup file.
cd /catkin_ws/                              # Go into catkin_ws for build.
catkin_make                                 # Build the workspace.
source devel/setup.bash                     # Source the setup file after the build.
chmod u+x src/sml_task/scripts/Prompter.py    # Make sure that the Prompter is an executable file. 
chmod u+x src/sml_task/scripts/Chatbot.py    # Make sure that the Chatbot is an executable file. 

roscore & # Run Master Node
sleep 15 # Give time for master node to spawn.
rosrun sml_task Prompter.py & # Run Prompter node in background.
rosrun sml_task Chatbot.py   # Run Chatbot node.
