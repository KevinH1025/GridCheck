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
DroneAI-Project/
│
├── px4/                   # PX4 source code (v1.14)
│
├── simulation/            # Simulation launch scripts
│   └── launch_sim.sh      # Starts PX4 + Gazebo simulation
│
├── drone_control/         # Python drone control & AI logic
│   ├── requirements.txt   # Python dependencies
│   ├── venv/              # Python virtual environment
│   │
│   ├── main.py            # Entry point script
│   │
│   ├── control/                 # drone control modules
│   │   ├── flight.py            # Arming, takeoff, landing, fight_to functions
│   │   ├── mission.py           # mission logic
│   │   └── battery_manager.py   # Battery monitoring & recharge logic (planned)
│   │   └── utils.py             # set ups for simulation (port, ...)
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
git clone https://github.com/yourusername/DroneAI-Project.git --recursive
cd DroneAI-Project
```

### 2. Install PX4 Autopilot (v1.14)
```bash
# Clone PX4 into the correct folder
git clone https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot --recursive
cd px4/PX4-Autopilot
git checkout v1.14.0  # Stable release for Gazebo 11

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
./launch_sim.sh
```

### 2. Run Drone Mission
```bash
# In a new terminal
cd drone_control
source venv/bin/activate 
python3 main.py
```

---

## Current Capabilities

- Autonomous takeoff
- Offboard mode flight
- Live telemetry feedback
- Position movement in NED frame
- Safe landing

---

## TODO / Future Work

- Implement grid search path planner
- Add YOLO AI detection module
- Implement battery monitoring & auto-recharge
- Deploy on real drone hardware





