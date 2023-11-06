#!/bin/bash

function run_command() {
    echo "Running: $1"
    eval $1
    if [ $? -ne 0 ]; then
        echo "Error executing: $1"
        exit 1
    fi
    echo "Command executed successfully."
    echo
}

echo "Starting YDLidar ROS2 Driver Installation..."

# Check for colcon 
if ! command -v colcon &> /dev/null; then
    run_command "sudo apt install python3-colcon-common-extensions"
fi

# Step 1: Create Workspace Directory
run_command "mkdir ~/ydlidar_ws"
run_command "cd ~/ydlidar_ws"

# Step 2: Clone YDLidar SDK Repository
run_command "git clone https://github.com/YDLIDAR/YDLidar-SDK.git"
run_command "cd YDLidar-SDK"

# Step 3: Build and Install YDLidar SDK
run_command "mkdir build"
run_command "cd build"
run_command "cmake .."
run_command "make"
run_command "sudo make install"
run_command "cd .."
run_command "pip install ."
run_command "cd build"
# run_command "cpack"
run_command "cd ~/"

# Step 4: Clone YDLidar ROS2 Driver Repository
run_command "mkdir ~/ydlidar_ws/src"
run_command "cd ~/ydlidar_ws/src"
run_command "git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git"

# Step 5: Build ROS2 Packages
run_command "cd ~/ydlidar_ws"
run_command "colcon build --symlink-install"
run_command "colcon build --symlink-install"

run_command "source ./install/setup.bash"

# Step 6: Set Up Environment Variables
run_command "echo 'source ~/ydlidar_ws/install/setup.bash' >> ~/.bashrc"
run_command "source ~/.bashrc"

# Step 7: Grant Execute Permissions to Startup Scripts
run_command "chmod 0777 src/ydlidar_ros2_driver/startup/*"

# Step 8: Run Initialization Script
run_command "sudo sh src/ydlidar_ros2_driver/startup/initenv.sh"

echo "YDLidar ROS2 Driver Installation Completed."
