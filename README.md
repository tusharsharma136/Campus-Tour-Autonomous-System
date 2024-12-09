# Multi Agent System Campus Virtual Tour - ROS2 Implementation

This project implements a **multi-agent system for a Campus Virtual Tour** using **ROS2 (Robot Operating System 2)**. The agents interact in a simulated campus environment, each performing specific tasks to create an immersive virtual tour for visitors. The system uses ROS2's powerful framework to simulate agent behaviors, enabling a realistic campus experience.

## Overview

The goal of the project is to develop a virtual tour system where multiple agents interact with each other in a virtual campus environment. These agents include:

- **BI Agent** (Building Information Agent): Provides detailed information about buildings and areas.
- **CI Agent** (Campus Infrastructure Agent): Helps navigate and interact with infrastructure systems.
- **Visitor Agent**: Simulates a visitor exploring the campus, interacting with other agents.

The project is built on ROS2, which provides essential tools for handling robot communication, simulation, and agent interaction. The agents work together to provide a cohesive and dynamic virtual campus tour experience.

## Project Structure

Here’s an overview of the project's folder structure:

- **build/**: Directory generated during the build process.
- **install/**: Contains installed project files after building the package.
- **launch/**: Contains the launch files for running the ROS2 simulation.
  - `campus_tour_simulation.launch.py`: Launch file to start the campus tour simulation.
- **log/**: Contains logs generated during the simulation.
- **resource/**: Directory for ROS2-specific resources (e.g., maps, models).
- **test/**: Contains test scripts and configurations.
- **campus_virtual_tour/**: Main folder containing all agent scripts.
  - `__init__.py`: Initializes the ROS2 package.
  - `bi_agent.py`: Script defining the behavior of the BI (Building Information) agent.
  - `ci_agent.py`: Script defining the behavior of the CI (Campus Infrastructure) agent.
  - `visitor_agent.py`: Script defining the behavior of the Visitor agent.
- **package.xml**: ROS2 package configuration file.
- **setup.cfg**: Configuration file for the Python package.
- **setup.py**: Python setup file for the ROS2 package.

## How to Run the Project in WSL (Windows Subsystem for Linux)

### Prerequisites

- **WSL 2**: Ensure that you are using WSL2 for better compatibility with ROS2. You can follow [this guide](https://docs.microsoft.com/en-us/windows/wsl/install) to install WSL2.
- **Ubuntu on WSL2**: Install Ubuntu from the Microsoft Store or use your preferred Linux distribution.
- **ROS2 Humble**: The project was built using ROS2 Humble, so make sure you install it using the following instructions:

  - Add ROS2 repository to your system:
    ```bash
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc
    echo "deb [arch=amd64] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    ```

  - Install ROS2 Humble:
    ```bash
    sudo apt install ros-humble-desktop
    ```

- **Python 3.x**: Ensure you have Python 3.x installed.

- **Install necessary dependencies**:

## Steps

### Step 1: Clone the Repository

```bash
git clone <repository-url>
cd campus_virtual_tour
```
### Step 2: Install ROS2 Dependencies

```bash
sudo apt install ros-humble-<dependency-name>
```
### Step 3: Build the ROS2 Workspace

```bash
colcon build
```
### Step 4: Source the Workspace Setup File

```bash
source install/setup.bash
```

### Step 5: Launch the Simulation

```bash
ros2 launch campus_virtual_tour campus_tour_simulation.launch.py
```

## Feel free to reach out in case of any queries:) 
