# ROS 1

## Install

### VirtualBox

If you prefer to use VirtualBox instead of a dedicated partition follow these steps.

- Download VirtualBox and install it.

- Download Ubuntu 20.04 LTS.

- Create a new virtual environment, select the iso, uncheck the unattended installation, select at leat 2 cores and 4096 mb, select NAT network adapter and bidirectional clipboard.

- Follow the installation steps and restart.

#### Fullscreen

Open the terminal and type:

```
sudo apt install build-essential gcc make perl dkms
```

Insert guest image, open it and follow the instructions.

Restart the virtual machine and resize the screen.

#### Shared folder

Open the terminal and type:

```
sudo adduser your_vm_username vboxsf
```

Shutdown the VM. In configuration select the shared folder path.

### Tools (optional)

```
sudo apt install terminator
sudo snap install code --classic
sudo apt install git
```

### ROS

Open the terminal and type:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```
sudo apt update
```

```
sudo apt install ros-noetic-desktop-full
```

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep python3-catkin-tools
```

Type:

```
gedit .bashrc
```

At the end of the file copy the next lines:

```
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
```

Save the file and reboot.

## Create a workspace

Open the terminal and type:

```
mkdir -p ros_ws/src
```

Note: a common name for the workspace is catkin_ws or ros_ws but it can be whatever you want, it just needs to be the same name as in the .bashrc file.

## Create a package

Move to the src directory:

```
cd ros_ws/src
```

Create a new package:

```
catkin_create_pkg name_of_the_package roscpp rospy std_msgs actionlib_msgs actionlib dependency_1
```

roscpp: dependency for C++.

rospy: dependency for Python.

std_msgs: common message types.

Note 1: in all cases above, dependencies are optional.

Note 2: C++ code must be inside ~/ros_ws/src/name_of_the_package/src folder.

Note 3: Python code must be inside ~/ros_ws/src/name_of_the_package/scripts folder.

Note 4: actionlib is just required if you want to use actions within the package. actionlib_msgs is required if you want to create a package with custom actions.

## Compile

Inside ros_ws compile all packages with:

```
catkin build
```

or just a selected package:

```
catkin build name_of_the_package
```

Note: every time you create a new package you need to resource with `source ~/ros_ws/devel/setup.bash` or relaunch the terminal.

## Nodes

### Terminal Commands

Type `roscore` inside an independent terminal before you run any node.

```
rosrun name_of_the_package name_of_the_executable
rosnode list
rosnode info /name_of_the_node
```

## Topics

### Terminal Commands

```
rostopic list
rostopic info /name_of_the_topic
rostopic echo /name_of_the_topic
rostopic hz /name_of_the_topic
rostopic pub -1 /name_of_the_topic /message_type "data: value"
rostopic pub --rate 1 /name_of_the_topic /message_type "data: value"
```

## Services

### Terminal Commands

```
rosservice list
rosservice info /name_of_the_service
rosservice call /name_of_the_service "request: value"
```

## Parameters

### Terminal Commands

```
rosparam list
rosparam set /name_of_the_parameter value
rosparam get /name_of_the_parameter
rosparam delete /name_of_the_parameter
```

## Actions

### Terminal Commands

```
rostopic pub -1 /name_of_the_action/goal name_of_the_interface "data: value"
rostopic echo /name_of_the_action/result
rostopic echo /name_of_the_action/feedback
rostopic echo /name_of_the_action/status
rostopic pub -1 /name_of_the_action/cancel actionlib_msgs/GoalID "data: value"
```

## Launch

### Terminal Commands

```
roslaunch name_of_the_package name.launch
```