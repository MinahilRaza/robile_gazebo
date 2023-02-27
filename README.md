# robile_gazebo

The first step is to install ROS. This code has been tested for ROS Noetic on Ubuntu 20.04. The default Python 3.8 installation has been used.

## ROS Installation
ROS Noetic can be installed using the following commands
~~~ sh
sudo apt-get update
sudo apt-get upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-catkin-tools python3-osrf-pycommon
~~~ 
The next step is to check if `rosdep` works correctly by running
~~~ sh
sudo rosdep init
~~~
If this command fails, some manual work arounds mentioned in this [link](https://lincoln-zhou.github.io/Install-ROS-on-Ubuntu-20.04.3/) can be used.
Lastly, run the following commands to finish the installation.
~~~ sh
rosdep update
mkdir -p ~/catkin_ws/src
~~~

## Package Installation
In addition to a few ROS dependencies, this package also depends on the following packages provided by KELO-robotics:
1. [kelo_tulip](https://github.com/kelo-robotics/kelo_tulip)
2. [robile_description](https://github.com/kelo-robotics/robile_description)

Eecute the below commands to install the simulator and its dependencies

~~~ sh
sudo apt install ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-effort-controllers ros-$ROS_DISTRO-velocity-controllers ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-control

cd ~/catkin_ws/src
git clone https://github.com/kelo-robotics/kelo_tulip.git
git clone https://github.com/kelo-robotics/robile_description.git
git clone https://github.com/kelo-robotics/robile_gazebo.git

catkin build kelo_tulip # you will need to enter your password for the kelo_tulip build to complete

catkin build robile_description robile_gazebo
source ~/catkin_ws/devel/setup.bash
~~~

## Controlling the Robot

To start the gazebo simulator, use the following launch file [launch/](launch/) directory as follows:

~~~ sh
source ~/catkin_ws/devel/setup.bash
roslaunch robile_gazebo 4_wheel_platform.launch
~~~

There are many different ways to control the robot. For example:
- by using a script which publishes to the Twist topic
- using the keyboard teleop script
- through the command line.

### Using a Python Script for Control
A simple Python file has been provided in this repository `robile_gazebo/src/python_tutorials/control_kelo.py`. First launch the simulator as described above. Then run,
~~~ sh
cd ~/catkin_ws/src/robile_gazebo/src/python_tutorials
python3 control_kelo.py
~~~

### Moving the Robot using a Keyboard

The [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) ROS package can be used to simplify controlling the motion of the robot. This requires a one time installation of the `teleop_twist_keyboard` package using the below command:

~~~ sh
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
source /opt/ros/$ROS_DISTRO/setup.bash
~~~

Then start the `teleop_twist_keyboard.py` node using the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` and move the robot as per the instructions displayed on the terminal. Below is an example of the output after running the node:

~~~
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
~~~

### Controlling through the Command Line
You can publish command velocities on the ros topic /cmd_vel to move the robot around. For example, to move the robot forward with 0.5 m/s velocity, execute the below command in a new terminal:

~~~ sh
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
  
~~~
