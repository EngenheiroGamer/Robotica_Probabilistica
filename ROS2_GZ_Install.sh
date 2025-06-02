#!/bin/bash
Ubuntu_Release=`lsb_release -r -s`
if [ ${Ubuntu_Release} == "22.04" ]; then
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade -y
    sudo pip3 install transforms3d
    sudo apt install ros-humble-desktop-full -y
    sudo apt install ros-dev-tools -y
    echo -e "export IGN_VERSION=fortress\nsource /opt/ros/humble/setup.bash" >> /home/${USER}/.bashrc
    source /home/${USER}/.bashrc
else
    echo "ROS2 Humble and Gazebo Fortress should be installed on Ubuntu 22.04! You're running from Ubuntu ${Ubuntu_Release}..."
fi