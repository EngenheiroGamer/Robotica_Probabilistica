# Robot Project for Robotics Course [CEFET-RJ]
A Robot project integrating ROS 2 and Gazebo (Fortress) simulator.

## Included packages

* `ros_gz_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_application` - holds ros2 specific code and configurations.

* `ros_gz_bringup` - holds launch files and high level utilities.


## Install Requirements

1. Download Project by Git Clone or using the Yaml
    $ git clone https://github.com/EngenheiroGamer/Robotica_Probabilistica.git
    or
    go to the folder you want to download the project (recomended ~/project_ws)
    if not created yet:
        $ mkdir ~/project_ws
        $ cd ~/project_ws
    $ wget https://raw.githubusercontent.com/EngenheiroGamer/Robotica_Probabilistica/main/workspace.yaml
    $ vcs import < workspace.yaml

## Install necessary tools and ROS2 Humble (If not installed yet)

    $ bash /path/to/downloaded/project/ROS2_GZ_Install.sh 

2. Build the project
    $ cd ~/project_ws
    $ bash InitProject.sh

3. Execute the project
    $ ros2 launch ros_gz_bringup robot.sdf

4. Move the robot
    In another terminal open the ROS2 teleop_twist_keyboard
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    use the keys showed on the menu to move the robot

5. Check on the RViz the sensors. You can also add the camera image by selecting by topics and Image option. 