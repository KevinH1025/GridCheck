# GridCheck

A complete autonomous drone system for grid search with AI detection and self-recharging logic.  
This project uses **PX4 SITL + Gazebo + MAVSDK + Python** to simulate, control, and automate missions.

---

## Features

- PX4 SITL simulation with Gazebo Classic
- MAVSDK Python integration
- Takeoff, landing & grid search using Offboard mode
- Real-time telemetry monitoring
- AI object detection module (YOLO - coming soon)
- Self-recharging logic (in development)

---

## Project Structure
```
GridCheck/
│
├── px4/                   # PX4 source code (v1.14)
│
├── simulation/            # Simulation launch scripts
│   └── sim.sh      # Starts PX4 + Gazebo simulation
│
├── drone_control/         # Python drone control & AI logic
│   ├── requirements.txt   # Python dependencies
│   ├── venv/              # Python virtual environment
│   │
│   ├── main.py            # Entry point script
│   │
│   ├── control/                 # drone control modules
│   │   ├── flight.py            # Arming, takeoff, landing, fly_to functions
│   │   ├── mission.py           # mission logic
│   │   └── battery_manager.py   # Battery monitoring & recharge logic (planned)
│   │   └── utils.py             # set ups for simulation (port, GPS ...)
│   │
│   ├── ai_module/         # YOLO detection module
│   │   └── detector.py    # Object detection logic (planned)
│               
├── README.md              # Project description and setup guide
│
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
# Clone PX4 into the correct folder
git clone https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot --recursive
cd px4/PX4-Autopilot

# Remove all untracked files and directories
git clean -fdx

# Reset any local changes
git reset --hard

# Get the right PX4 version
git checkout v1.14.0  # Stable release for Gazebo 11

# Update submodules
git submodule update --init --recursive

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
```bash
# In a new terminal
cd drone_control
source venv/bin/activate 
python3 main.py
```

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
     - Click **Add** or **Edit**, set type to **UDP**
     - Set the listening port to `14551`
     - Click **Connect**
     - Then follow the next step

3. **Update PX4 MAVLink Port**

If QGC still doesn’t detect PX4, your PX4 instance may be sending on an unexpected port like `18570`.
In that case, modify the PX4 startup script `px4-rc.mavlink`:
```bash
# Navigate to the MAVLink config script
cd px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/

# Open the file
nano px4-rc.mavlink

# Add:
mavlink stop -u 18570 # Stop default instance 0

# Start your custom instance 0
mavlink start -u 14551 -p -m onboard -r 4000000
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
You should see 4 instances. Make sure: Instance 0 is bound to port `14551` and Broadcast is enabled. If not, restart the MAVLink instance manually:
```bash
# Replace PORT with the actual port (e.g. 18570)
mavlink stop -u PORT
mavlink start -u 14551 -p -m onboard -r 4000000
```
You’ll need to do this manually each time unless the PX4 startup config is properly updated.

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





