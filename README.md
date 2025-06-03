# Robot Project for Robotics Course [CEFET-RJ]<br />
A Robot project integrating ROS 2 and Gazebo (Fortress) simulator.<br />
<br />
## Included packages<br />
<br />
* `ros_gz_description` - holds the sdf description of the simulated system and any other assets.<br />
<br />
* `ros_gz_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.<br />
<br />
* `ros_gz_application` - holds ros2 specific code and configurations.<br />
<br />
* `ros_gz_bringup` - holds launch files and high level utilities.<br />
<br />
<br />
## Install Requirements<br />
<br /><br />
1. Download Project by Git Clone<br />
    $ git clone https://github.com/EngenheiroGamer/Robotica_Probabilistica.git<br />
<br />
## Install necessary tools and ROS2 Humble (If not installed yet)<br />
<br />
    $ cd /path/to/downloaded/project/Robotica_Probabilistica<br />
    $ bash ROS2_GZ_Install.sh <br />
<br />
2. Build the project and source<br />
    $ cd /path/to/downloaded/project/Robotica_Probabilistica<br />
    $ bash InitProject.sh<br />
    $ source install/setup.sh<br /> 
<br />
3. Execute the project<br />
    $ ros2 launch ros_gz_bringup robot.launch.py<br />
<br />
4. Move the robot<br />
    In another terminal open the ROS2 teleop_twist_keyboard<br />
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard<br />
    use the keys showed on the menu to move the robot<br />
<br />
5. Use the A* Algorithm:<br />
$ cd /path/to/downloaded/project/Robotica_Probabilistica<br />
$ python3 TrabalhoFinal_RoboticaProbabilistica.py<br />
<br />
The grid created is displayed underground, without affecting the moviment of the car, and the goal is to move the car from one side to the other through the grid, using the sortest possible path, without touching the wall. At the end, the openCV image opens, showing the box identified by the green line around it.<br />