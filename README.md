# ARAP Robot

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

## Overview
ARAP Robot is a ROS 2-based mobile robot project that combines navigation, localization, and simulation capabilities. This repository contains all the necessary packages to run and control the ARAP robot in both simulation and real-world environments.

## Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Git

## Dependencies
The following ROS 2 packages are required:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-urdf-tutorial
```

## Installation

1. **Install ROS 2 Jazzy**
   Follow the official ROS 2 Jazzy installation guide for Ubuntu 24.04:
   ```bash
   # Add ROS 2 apt repository
   sudo apt update && sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS 2 Jazzy
   sudo apt update
   sudo apt install ros-jazzy-desktop
   ```

2. **Install Gazebo Harmonic**
   ```bash
   sudo apt install gazebo-harmonic
   ```

3. **Clone the Repository**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/akooku/arap_robot.git
   ```

4. **Build the Workspace**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

## Setup

1. **Source the Setup Files**
   Add the following lines to your `~/.bashrc` file:
   ```bash
   # ROS 2 setup
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Add Convenience Aliases**
   Add these aliases to your `~/.bashrc` file for easier operation:
   ```bash
   # Build workspace
   alias build='cd ~/ros2_ws/ && colcon build && source ~/.bashrc && source install/setup.bash'
   
   # Launch robot visualization
   alias arap='ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/src/arap_robot/arap_robot_description/urdf/robots/arap_robot.urdf.xacro'
   
   # Launch Gazebo simulation
   alias arap_robot='bash ~/ros2_ws/src/arap_robot/arap_robot_bringup/scripts/arap_robot_gazebo.sh'
   
   # Launch navigation
   alias nav='bash ~/ros2_ws/src/arap_robot/arap_robot_bringup/scripts/arap_navigation.sh'
   ```

   After adding these lines, source your bashrc:
   ```bash
   source ~/.bashrc
   ```

## Usage

1. **Build the Project**
   ```bash
   build
   ```

2. **Launch the Robot in Gazebo**
   ```bash
   arap_robot
   ```
   This will launch the robot in a simulated environment with all necessary components.

3. **Launch Navigation**
   ```bash
   nav
   ```
   This will start the navigation stack, including:
   - SLAM for mapping (nav slam)
   - Navigation2 for path planning
   - Localization using AMCL

4. **View Robot Model**
   ```bash
   arap
   ```
   This will open RViz with the robot model for visualization.

## Project Structure
- `arap_robot_bringup`: Launch files and scripts for starting the robot
- `arap_robot_description`: URDF and mesh files for the robot model
- `arap_robot_gazebo`: Gazebo simulation configurations
- `arap_robot_navigation`: Navigation stack configurations
- `arap_robot_localization`: Localization packages
- `arap_robot_system_tests`: System test packages

## Troubleshooting

### Common Issues

1. **Build Errors**
   - Ensure all dependencies are installed
   - Try cleaning the build: `cd ~/ros2_ws && rm -rf build/ install/ log/`
   - Rebuild: `colcon build`

2. **Launch Failures**
   - Check if ROS 2 environment is sourced: `printenv | grep -i ROS`
   - Verify workspace is built and sourced
   - Check launch file permissions: `chmod +x ~/ros2_ws/src/arap_robot/arap_robot_bringup/scripts/*.sh`

3. **Navigation Issues**
   - Ensure map is properly loaded
   - Check if robot's initial pose is set correctly
   - Verify sensor data is being published

### Getting Help
If you encounter any issues:
1. Check the [ROS 2 documentation](https://docs.ros.org/en/jazzy/)
2. Search existing [GitHub issues](https://github.com/akooku/arap_robot/issues)
3. Create a new issue with detailed information about your problem

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request. Before contributing:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact
- GitHub: [@akooku](https://github.com/akooku)
- Email: [akooku12](akooku12@gmail.com)
