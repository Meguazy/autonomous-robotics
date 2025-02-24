# autonomous_robotics
## Installation :arrow_down:

### Requirements

This control system requires a system setup with ROS 2. It is recommended to use Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html), however using Ubuntu 20.04 with [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html) should also work.

### Install ROS 2 Humble Hawksbill
Follow the instructions from the [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). There are 3 versions of ROS 2 Humble Hawksbill to choose from: Desktop Install, ROS-Base Install and Development tools Install. Be sure to install **Desktop Install** version (`sudo apt install ros-humble-desktop`).


### Install additional modules and packages  
All necessary modules are in [requirements.txt](https://github.com/jkaniuka/magician_ros2/blob/main/requirements.txt), install using: `pip3 install -r requirements.txt`   
Packages from apt repository: `sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl`    
:warning: After installing new RQT plugins run `rqt --force-discover` to make plugins visible in RQT GUI. This issue is further described [here](https://answers.ros.org/question/338282/ros2-what-is-the-rqt-force-discover-option-meaning/).

### Create workspace for control system (build from source)
```
source /opt/ros/humble/setup.bash
mkdir -p ~/magician_ros2_control_system_ws/src
git clone https://github.com/jkaniuka/magician_ros2.git ~/magician_ros2_control_system_ws/src
cd magician_ros2_control_system_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```
