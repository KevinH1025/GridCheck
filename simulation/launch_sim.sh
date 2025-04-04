#!/bin/bash
cd "$(dirname "$0")/../px4/PX4-Autopilot"
make px4_sitl_default gazebo  # Launches PX4 + Gazebo
