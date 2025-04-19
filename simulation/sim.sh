#!/bin/bash
cd "$(dirname "$0")/../px4/PX4-Autopilot"
make px4_sitl gazebo-classic_iris  # Launches PX4 + Gazebo + default drone
# make px4_sitl gazebo-classic_iris # Launches PX4 + Gazebo + stereo camera drone