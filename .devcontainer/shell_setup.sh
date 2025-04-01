export WS="ako_ws"

export DEV_DIR="/home/ubuntu/Development"
export ROS2_DISTRO='jazzy'

SHELL=$1

alias rosargcomp='eval "$(register-python-argcomplete ros2)" && eval "$(register-python-argcomplete colcon)"'
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/$ROS2_DISTRO/

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$SHELL
source /opt/ros/$ROS2_DISTRO/setup.$SHELL
source $DEV_DIR/$WS/install/setup.$SHELL

alias sr='source ~/.bashrc'
alias snb='sudo nano ~/.bashrc'

alias cbs='rd && colcon build --symlink-install && sr'
alias cb='rd && colcon build && sr'
alias snm='nano $DEV_DIR/scripts/shell_setup.sh'
alias rd='cd $DEV_DIR/$WS'
alias rds='cd $DEV_DIR/$WS/src'
alias rdd='cd $DEV_DIR/$WS/src/$DEV_FOLDER'

alias drd='rd && sudo rm -rf build log install'

alias cbp='rd && colcon build --packages-select'
alias cbsp='rd && colcon build --symlink-install --packages-select'
alias ri='rd && rosdep install --from-paths src --ignore-src -y -r'
alias rr='ros2 run'
alias rl='ros2 launch'
alias rtl='ros2 topic list'
alias rnl='ros2 node list'
alias rte='ros2 topic echo'

alias ct='rd && colcon test --packages-select'

# ROS2 Package Creation for Python and C++
alias rpp="rdd && ros2 pkg create --dependencies rclpy --build-type ament_python"
alias rpc="rdd && ros2 pkg create --dependencies rclcpp --build-type ament_cmake"

alias gsu='git submodule init && git submodule update'
alias gc='git clone'
alias ud='sudo apt-get update -y'
alias ug='sudo apt-get upgrade -y'
alias udg='ud && ug'
alias si='sudo apt-get install -y'

# Bind autocompletion to aliases
complete -F _ros2 rr
complete -F _ros2 rl

echo "ROS2 Workspace: $WS"
