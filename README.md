# Autonomous and collaborative robotics: Simulating multi-robot collaboration
## Installation :arrow_down:

### Requirements

This control system requires a system setup with ROS 2. It is recommended to use Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html). All of the software must be used on a Raspberry Pi connected to a webcam and on the same wifi network of the robotic arm. Also, the user has to know the robotic arm's IP in order to correctly modifiy the IP inside 
```
src/dovot_driver/dovot_driver/dobot_handle.py
```
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

## System startup :robot:
1. Connect Dobot Magician on the same network of the Raspberry and then turn it on. 
2. Set the MAGICIAN_TOOL environment variable describing the robot's configuration `export MAGICIAN_TOOL=<tool_type>` (allowed values are: _none, pen, suction_cup, gripper, extended_gripper_).
3. From inside of the **magician_ros2_control_system_ws** directory, run `. install/setup.bash` to source your workspace.
3. Launch entire control stack with `ros2 launch dobot_bringup dobot_magician_control_system.launch.py`. 


<a name="homing"></a>
## Homing procedure
Homing should be performed as the first action after the system is started. It is necessary because an incremental encoder has been placed in the base of the manipulator, and the robot is not aware of its actual position when it is powered up. Stop all other scripts controlling the robot before starting the homing procedure.   
Homing is handled by the service server, to start it run the following command:
```
ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure
```

## Publish Green Light on Semaphore and Trigger Action Server

The following command is used to publish a "green" signal to the `/semaphore` topic, which allows the action server to start its execution:

```bash
ros2 topic pub --once /semaphore std_msgs/msg/String "{data: 'green'}"

