#!/bin/bash

# Script to set up ROS2 environment with Apex-Putter and Franka workspace
# This script sources the necessary setup files for the project

# Function to check if sourcing was successful
source_and_check() {
    local setup_file="$1"
    local description="$2"
    
    if [ ! -f "$setup_file" ]; then
        echo "Error: $description setup file not found at: $setup_file"
        return 1
    fi
    
    echo "Sourcing $description..."
    . "$setup_file"
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to source $description"
        return 1
    fi
    
    echo "Successfully sourced $description"
    return 0
}

# Directory paths
APEX_PUTTER_DIR="$HOME/me495/final_project/Apex-Putter"
FRANKA_WS_DIR="$HOME/ws/franka"
VENV_ROS_DIR="$HOME/venv_ros"

# Source Apex-Putter setup
if ! source_and_check "$APEX_PUTTER_DIR/install/setup.bash" "Apex-Putter"; then
    exit 1
fi

# Source Franka workspace setup
if ! source_and_check "$FRANKA_WS_DIR/install/setup.bash" "Franka workspace"; then
    exit 1
fi

# Source ROS virtual environment setup
if ! source_and_check "$VENV_ROS_DIR/install/setup.bash" "ROS virtual environment"; then
    exit 1
fi

echo "All environments successfully sourced!"