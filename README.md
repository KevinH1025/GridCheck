# GridCheck

A complete autonomous drone system for grid search with AI detection and self-recharging logic.  
This project uses **PX4 SITL + Gazebo + ROS2 + MAVROS2 + Python/C++** to simulate, control, and automate missions.

---

## Features

- PX4 SITL simulation with Gazebo Classic
- ROS2 Python/C++ integration
- Takeoff, landing & navigation using Offboard mode
- Real-time telemetry monitoring
- AI object detection module (YOLO - coming soon)
- Self-recharging logic (in development)

---

## Project Structure
```
GridCheck/
├── px4/                   # PX4 source code (v1.14)
├── ros2_ws/               # ROS 2 workspace packages
│   └── src/                
│       ├── drone_control/       # Drone control package
│       │   ├── drone_control/           
│       │   │   ├── drone_control.py     # Handles high-level commandes      
│       │   │   ├── mission_planner.py   # Manages takeoff, arming, landing and task sequencing
│       │   │   └── state_monitor.py     # Gives the currrent drone state
│       │   └── set_up.py        # ROS to initialize package
│       │         
│       └── simulation/          # Simulation package -> starting
│           ├── simulation/              
│           │   └── mavros2_launch.py    # Custom launch file to start MAVROS 2 and packages          
│           └── set_up.py        # ROS to initialize package
├── venv/                  # Python virtual environment            
├── README.md              # Project description and setup guide
├── requirements.txt       # Python dependencies
└── .gitignore             # Git ignore rules
```

---

## Setup Instructions

### Prerequisites
- Ubuntu 20.04 (WSL2 recommended for Windows users)
- Python 3.8+

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/GridCheck.git --recursive
cd GridCheck
```

### 2. Install PX4 Autopilot (v1.14)
```bash
# Clone the stable PX4 v1.14.0 in into the correct folder 
git clone --branch v1.14.0 --recursive https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot

# Install dependencies
bash ./Tools/setup/ubuntu.sh
```

### 3. Set Up Gazebo 11
```bash
sudo apt-add-repository universe
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev

# Verify
gazebo --version  # Should show "gazebo 11.X.X"
```

### 4. Set up ROS 2
```bash
# Add ROS 2 source
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Foxy
sudo apt update
sudo apt install -y ros-foxy-desktop

# Source ROS 2
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Set up MAVROS 2
```bash
# Navigate to right folder
cd GridCheck/ros2_ws/src

# Clone MAVROS 2
git clone https://github.com/mavlink/mavros.git -b ros2
vcs import < mavros/mavros2.repos

# Path install?
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build everything
colcon build --symlink-install

# Source MAVROS 2
source ~/GridCheck/ros2_ws/install/setup.bash
```

### Set Up Python environment 
```bash
# Install Python dependencies 
sudo apt update
sudo apt install -y python3-pip python3-venv

# Create and activate virtual environment
cd drone_control
python3 -m venv venv
source venv/bin/activate

# Install Python packages
pip install -r requirements.txt

# If not using VScode, activate venv before running the project
source venv/bin/activate
```

---

##  Start Simulation

### 1. Launch PX4 + Gazebo
```bash
cd simulation
./sim.sh
```

### 2. Run Drone Mission

Comming soon

---

## QGroundControl (Recommended)
QGroundControl is a powerful open-source ground control station for MAVLink-compatible drones, 
used to monitor telemetry, view flight status, and send mission commands in real time.

### Setup Instructions

1. **Download QGroundControl**  
   Download the latest version from:  
   [https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)
   
2. **Connect to PX4 SITL**
   - Launch QGroundControl and simulation
   - By default, it listens on UDP port `14550`
   - If the connection works, you're good to go
   - If nothing shows up within a few seconds:
     - Go to **Settings → Comm Links**
     - Check **Automatically Connect on Start** box
     - Click **Add**, set type to **UDP**
     - Set port to `14551`
     - Click **Connect**
     - Then follow the next step

3. **Update PX4 MAVLink Port (WSL2 + Windows QGroundControl)**

   If QGroundControl is running on **Windows** and PX4 is running in **WSL2 (Ubuntu)**, the two systems are on different virtual networks.  
   This means PX4's MAVLink messages won't reach QGroundControl unless you explicitly set the Windows IP address as the target.
   
   In **Ubuntu** run:
   ```bash 
   cat /etc/resolv.conf | grep nameserver
   ```
   
   You’ll get something like:
   ```bash 
   nameserver 172.27.96.1
   ```
   
   This is your Windows host IP from inside WSL2. Now modify the PX4 startup script `px4-rc.mavlink`:
   ```bash
   # Navigate to the MAVLink config script
   cd px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
   
   # Open the file
   nano px4-rc.mavlink
   
   # Add:
   mavlink stop -u 18570 # Stop default instance (usually on port 18570)
   
   # Start your custom instance 0 (sending to QGroundControl on Windows)
   mavlink start -u 14551 -t 172.27.96.1 -p -m onboard -r 4000000
   ```

4. **Restart the Simulation**

   ```bash
   cd simulation
   ./sim.sh
   ```
   QGroundControl should now automatically connect to the drone and display it on the map. If it still doesn't connect, open the PX4 shell and check MAVLink status:
   ```bash
   mavlink status
   ```
   You should see 4 instances. Make sure: Instance 0 is bound to port `14551` and Broadcast is enabled. If not, restart the MAVLink instance manually, in PX4 shell run:
   ```bash
   # Replace PORT with the actual port (e.g. 18570)
   mavlink stop -u PORT
   mavlink start -u 14551 -t 172.27.96.1 -p -m onboard -r 4000000
   ```
   You’ll need to do this manually each time unless the PX4 startup config `px4-rc.mavlink` is properly updated.
   
   ---

## Current Capabilities

- Autonomous takeoff
- Offboard mode flight
- Live telemetry feedback
- Position movement in NED and GPS frame
- Safe landing

---

## TODO / Future Work

- Implement grid search path planner
- Add YOLO AI detection module
- Implement battery monitoring & auto-recharge
- Deploy on real drone hardware





