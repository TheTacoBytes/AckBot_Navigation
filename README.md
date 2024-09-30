# Ackbot Project

## Overview
The AckBot project provides the necessary code and configurations to control and navigate a robot using a Raspberry Pi 4 (AckBot) and a laptop running Ubuntu 22 with ROS 2 Humble. AckBot itself handles the basic sensor and movement operations, such as publishing topics like `cmd_vel`, `odom`, and `scan`. However, all higher-level functions, such as mapping (SLAM), localization (AMCL), and navigation (Nav2), are handled on the laptop, which subscribes to these topics.

### Key Points:

- **AckBot (Raspberry Pi 4)**: Runs basic ROS 2 nodes, publishing necessary data to topics for movement and sensing (cmd_vel, odom, scan).
- **Laptop (Ubuntu 22, ROS 2 Humble)**: Handles more computationally intensive tasks, such as controlling AckBot with a PS4 controller, running SLAM for mapping, and using AMCL and Nav2 for path planning and navigation.
- **Multi-Machine Setup**: Both the Raspberry Pi 4 and the laptop are set to the same ROS_DOMAIN_ID to ensure proper communication across the network.

This repository focuses on setting up and running the SLAM and navigation tasks from the laptop, with control over AckBot via a PS4 controller. All the tasks involving teleoperation, SLAM, saving the map, and performing autonomous navigation are executed on the laptop while AckBot serves as the robot's hardware platform.

### Table of Contents:
1. [Dependencies](#dependencies)
2. [Setup](#setup)
3. [Running Ackbot](#running-ackbot)
4. [Performing SLAM and Saving Maps](#performing-slam-and-saving-maps)
5. [Running AMCL and Navigation](#running-amcl-and-navigation)
   
## Dependencies

- **ROS 2 Humble**: The laptop uses Ubuntu 22 and ROS 2 Humble to run SLAM, AMCL, and navigation.
- **Ackbot's Pi 4**: Ackbot is powered by a Raspberry Pi 4, which publishes the topics `cmd_vel`, `odom`, and `scan` to which the laptop subscribes.
- **Nav2**: For path planning and navigation, which runs on the laptop.
- **SLAM Toolbox**: For scanning the room and generating maps (runs on the laptop).
- **PS4 Controller Node**: For teleoperating Ackbot using the PS4 controller (also controlled from the laptop).

## Setup

### Clone the Repository
To set up the Ackbot navigation stack, first create a new ROS 2 workspace on the **laptop** if you haven't already:

```bash
mkdir -p ~/ackbot_ws/src
cd ~/ackbot_ws/src
git clone https://github.com/TheTacoBytes/AckBot_Navigation.git .
```

### Build the Workspace

After cloning the repository, navigate to the workspace root and build it using `colcon`:
```bash
cd ~/ackbot_ws
colcon build
```

### Source the Workspace

After building, make sure to source the workspace on the laptop:

```bash
source install/setup.bash
```

## Running SLAM and Saving Maps
The following steps will require the AckBot to be powered on and connected to same Wi-Fi as the Laptop. 

### 1. Multi-Machine Required
First, In order for the laptop to see the topics you need to make sure you have set both AckBot and Laptop on the same `ROS_DOMAIN_ID`. For example, you can set it to `1` by running:

```bash
export ROS_DOMAIN_ID=1
```
Now check to see if the Laptop can see the topics from the AckBot's Pi by running:
```bash
ros2 topic list
```
There you should see `cmd_vel`,`scan`,`odom`, ect.

### 2. Launch the PS4 Controller Node to Control Ackbot Manually

Next, you need to launch the PS4 controller node to teleoperate Ackbot for scanning the environment.
```bash
ros2 launch ps4_controller controller_launch.py
```
### 3. Launch the SLAM Node to Perform Scanning
Once you have control of Ackbot, run the SLAM node on your laptop to scan the environment:
```bash
ros2 launch ackbot_slam_launch slam_launch.py
```
This will initiate SLAM, and Ackbot will start mapping the environment. Use the PS4 controller to move Ackbot around to create a complete map.

Once the scanning is complete, use the **SLAM panel** in your interface to save the map, so you dont have to edit the code reuse the name `map`. The map is saved as `map.pgm` and `map.yaml` on the **laptop**.

### 4. Move the Map for Navigation  
Move the saved map files to the appropriate folder for Nav2 to use:
```bash
cd ~/ackbot_ws
mv map.pgm map.yaml ~/ackbot_ws/src/ackbot_nav2_bringup/maps/
```
You can now cancel the controller process and the slam process from running.

Once finished you will need to rebuild the workspace and source it one more time.
After cloning the repository, navigate to the workspace root and build it using `colcon`:
```bash
cd ~/ackbot_ws
colcon build
source install/setup.bash
```

## Running AMCL and Navigation
### Localization with AMCL 
To localize Ackbot in the environment using the saved map, run the following command on the **laptop**:
```bash
ros2 launch ackbot_nav2_bringup localization_launch.py use_sim_time:=false
```
Now based on where the AckBot is in the realworld set a 2D Pose Estimate in RVIZ in the map. If you dont see the map in RVIZ look at all the **map** drop downs and find the one that has the topic `/map` change it to anything else and change it back. Then, go into the **topic** drop down and set the **Durability Policy** to `Transient Local`.

### Path Planning and Navigation
Once Ackbot is localized, you can run Nav2 for path planning and navigation on the **laptop**:
```bash
ros2 launch ackbot_nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
```

### Set Goal Pose
In RVIZ click 2D Goal Pose and set the position and direction and watch as the AckBot moves.
License  
This project is licensed under the Apache License 2.0.

