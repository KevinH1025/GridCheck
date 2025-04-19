import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory
from .config import WAYPOINT_REACH_ALTITUDE, WAYPOINT_REACH_DISTANCE

# load mission from yaml
def load_mission(filename):
    path = os.path.join(
        get_package_share_directory('drone_control'),  # your actual ROS 2 package name
        filename
    )
    with open(path, 'r') as f:
        return yaml.safe_load(f)['waypoints']

# all waypoints to gps coordinates -> TODO: Use better conversion, more accurate
def convert_mission(current_lat, current_lon, mission):
    wp_list = []
    for dx, dy, dz in mission:
        delta_lat = dx / 111320.0
        delta_lon = dy / (111320.0 * math.cos(math.radians(current_lat)))
        new_lat = float(current_lat + delta_lat) 
        new_lon = float(current_lon + delta_lon)
        wp_list.append((new_lat, new_lon, float(dz)))
    return wp_list

# check if the waypoint is reached
def reached(wp, curr_lat, curr_lon, rel_alt):
        lat, lon, alt = wp
        lat_err = (lat - curr_lat) * 111320
        lon_err = (lon - curr_lon) * 111320 * math.cos(math.radians(lat))
        alt_err = alt - rel_alt
        return math.sqrt(lat_err**2 + lon_err**2) < WAYPOINT_REACH_ALTITUDE and abs(alt_err) < WAYPOINT_REACH_DISTANCE