# Multi-Drone Formation Control with ROS and Gazebo

## Overview
This project demonstrates the control of a swarm of four drones in a simulated environment using ROS, Gazebo, MAVROS, and ArduPilot SITL. The drones perform coordinated movements in specific formations, utilizing waypoints defined in custom C++ scripts.

## Features
- **Multi-Drone Simulation**: Four drones simulated using ArduPilot SITL and Gazebo.
- **Waypoint Navigation**: Each drone navigates predefined waypoints, maintaining formation.
- **Formation Control**: Custom C++ scripts control the drones to move in patterns like squares, rectangles, or circles.
- **Automated Launch**: A bash script automates launching the simulation, MAVROS nodes, and control scripts.

## Technologies Used
- **ROS Noetic**: Middleware for robotic systems.
- **Gazebo**: 3D simulation environment.
- **ArduPilot SITL**: Software-in-the-loop simulation for drones.
- **MAVROS**: Communication between ROS and MAVLink-compatible drones.
- **C++**: Setpoint scripts for controlling drone movements.

---

## Prerequisites
- **Operating System**: Ubuntu 20.04
- **Software Requirements**:
  - ROS Noetic
  - Gazebo (compatible version with ROS Noetic)
  - ArduPilot SITL
  - MAVROS and MAVROS-Extras

### Install Required Packages
1. Update your system and install ROS Noetic:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

2. Install MAVROS and MAVROS Extras:
   ```bash
   sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
   ```

3. Install GeographicLib datasets for MAVROS:
   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo bash install_geographiclib_datasets.sh
   ```

4. Ensure Gazebo and ArduPilot SITL are installed:
   - Follow [Gazebo Installation Guide](http://gazebosim.org/)
   - Follow [ArduPilot SITL Installation Guide](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

---

## Setup

### Clone the Repository
```bash
git clone https://github.com/yourusername/multi_drone_control.git
cd multi_drone_control
```

### Build the Workspace
1. Create a new ROS workspace and copy the project:
   ```bash
   mkdir -p ~/catkin_ws/src
   cp -r multi_drone_control ~/catkin_ws/src/
   ```

2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## Usage

### Launch the Simulation
Use the provided bash script to initialize all components in separate terminals:
```bash
./scripts/run_simulation.sh
```
This script will:
1. Launch Gazebo with a multi-drone world.
2. Start SITL instances for each drone.
3. Launch MAVROS nodes for communication.
4. Run custom setpoint control scripts for all drones.

### Monitor Drones
1. View the simulation in Gazebo.
2. Use `rostopic` to monitor MAVROS topics:
   ```bash
   rostopic echo /uav1/mavros/state
   ```

### Customizing Waypoints
To modify the drone trajectories:
1. Edit the respective setpoint scripts in the `scripts/` directory (e.g., `setpoint_node1.cpp`).
2. Rebuild the workspace after making changes:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

---

## Repository Structure
```plaintext
multi_drone_control/
├── README.md               # Project documentation
├── scripts/                # Bash and C++ scripts
│   ├── run_simulation.sh   # Script to automate simulation setup
│   ├── setpoint_node1.cpp  # Setpoint control script for Drone 1
│   ├── setpoint_node2.cpp  # Setpoint control script for Drone 2
│   ├── setpoint_node3.cpp  # Setpoint control script for Drone 3
│   ├── setpoint_node4.cpp  # Setpoint control script for Drone 4
├── worlds/                 # Gazebo world files
│   ├── iris_multiuav.world # Multi-drone simulation world
├── launch/                 # Launch files for ROS
│   ├── multi.launch        # Launch file for multi-drone Gazebo world
│   ├── mavros_apm.launch   # MAVROS launch file template
├── docs/                   # Documentation assets
│   ├── images/             # Screenshots and figures
├── LICENSE                 # Project license
└── .gitignore              # Ignored files
```

---

## Contribution Guidelines
1. Fork the repository and create a feature branch for your changes.
2. Commit changes with meaningful messages.
3. Create a pull request for review.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Acknowledgements
- [ROS Documentation](https://wiki.ros.org/)
- [ArduPilot Documentation](https://ardupilot.org/)
- [Gazebo Documentation](http://gazebosim.org/)

