FROM osrf/ros:jazzy-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Install additional utilities
RUN sudo apt-get install -y nano wget curl

# Create workspace directories
# CHANGE THIS TO YOUR OWN WORKSPACE NAME
RUN mkdir -p /home/ubuntu/Development/ako_ws/src
RUN mkdir -p /home/ubuntu/Development/scripts

# Copy setup script
COPY --chown=ubuntu:ubuntu shell_setup.sh /home/ubuntu/Development/scripts/shell_setup.sh

# Source the shell setup script in .bashrc
RUN echo "source /home/ubuntu/Development/scripts/shell_setup.sh bash" >> ~/.bashrc

# Install ROS 2 dependencies packages
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt-get install -y ros-$ROS_DISTRO-ament-cmake-black ros-$ROS_DISTRO-ament-black
RUN sudo apt-get install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-joint-state-broadcaster ros-$ROS_DISTRO-teleop-twist-keyboard
RUN sudo apt-get install -y ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui
RUN sudo apt-get install -y ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-ros-gz-bridge ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-image ros-$ROS_DISTRO-sensor-msgs
RUN sudo apt-get install -y ros-$ROS_DISTRO-rqt-image-view ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-gz-*