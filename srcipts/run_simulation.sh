#!/bin/bash

# Function to open a new terminal and run a command
run_in_new_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

# Launch Gazebo
run_in_new_terminal "roslaunch gazebo_ros multi.launch"
sleep 15  # Wait for 15 seconds after launching Gazebo

# Start ArduCopter SITL instances
run_in_new_terminal "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I0"
sleep 2  # Wait for 2 seconds
run_in_new_terminal "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I1"
sleep 2
run_in_new_terminal "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I2"
sleep 2
run_in_new_terminal "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I3"
sleep 2

# Launch MAVROS for each drone
run_in_new_terminal "ROS_NAMESPACE=uav1 roslaunch mavros apm.launch fcu_url:=\"udp://:14550@127.0.0.1:14550\""
sleep 2
run_in_new_terminal "ROS_NAMESPACE=uav2 roslaunch mavros apm.launch fcu_url:=\"udp://:14560@127.0.0.1:14560\""
sleep 2
run_in_new_terminal "ROS_NAMESPACE=uav3 roslaunch mavros apm.launch fcu_url:=\"udp://:14570@127.0.0.1:14570\""
sleep 2
run_in_new_terminal "ROS_NAMESPACE=uav4 roslaunch mavros apm.launch fcu_url:=\"udp://:14580@127.0.0.1:14580\""
sleep 60

# Run control nodes for each drone
run_in_new_terminal "rosrun drone_control setpoint_node_t1"
sleep 2
run_in_new_terminal "rosrun drone_control setpoint_node2"
sleep 2
run_in_new_terminal "rosrun drone_control setpoint_node3"
sleep 2
run_in_new_terminal "rosrun drone_control setpoint_node4"
