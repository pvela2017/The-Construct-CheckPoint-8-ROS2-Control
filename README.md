# Checkpoint 8

<a name="readme-top"></a>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#software-prerequisites">Software Prerequisites</a></li>
        <li><a href="#hardware-prerequisites">Hardware Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage-ranger-robot">Usage Ranger Robot</a></li>
  </ol>
</details>


## About The Project
Package for simulating a differential type robot including ROS2 differential controller and an effort controller for the platform.


<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* ROS2 Humble

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo
   ```sh
   git clone https://github.com/pvela2017/checkpoint8
   ```
2. Compile
   ```sh
   cd ~/ros2_ws && colcon build && source install/setup.bash
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE of the ROBOT -->
## Usage Robot
1. Launch the simulation and activate the controllers:
```sh
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
```

2. Move the robot:
```sh
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
```

3. Lift the platform:
```sh
ros2 topic pub --rate 10 /elevator_controller/commands std_msgs/msg/Float64MultiArray  "data:
- 10.0"
```

4. Lower the platform:
```sh
ros2 topic pub --rate 10 /elevator_controller/commands std_msgs/msg/Float64MultiArray  "data:
- -10.0"
```

5. Check controllers and hardware interfaces:
```sh
ros2 control list_hardware_interfaces
ros2 control list_controllers
```
