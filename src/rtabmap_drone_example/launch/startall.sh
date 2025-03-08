#!/bin/bash
gnome-terminal -- bash -c "roslaunch rtabmap_drone_example gazebo.launch; exec bash"
gnome-terminal -- bash -c "roslaunch rtabmap_drone_example slamlatest.launch; exec bash"
gnome-terminal -- bash -c "roslaunch rtabmap_drone_example rviz.launch; exec bash"
