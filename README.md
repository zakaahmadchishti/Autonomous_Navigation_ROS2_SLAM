# Autonomous Navigation Theory and Application Using SLAM with ROS 2

**Authors:**  
- Zaka Ahmed, Master of Engineering (Information Technology), Frankfurt University of Applied Sciences  
- Muhammad Haris, Master of Engineering (Information Technology), Frankfurt University of Applied Sciences  

---

## Abstract

This project implements autonomous navigation for a **TurtleBot 3** using the **ROS 2 Navigation Stack (Nav2)**. It integrates **Simultaneous Localization and Mapping (SLAM)** to build an occupancy grid map, while using Nav2 for localization, path planning, and execution. The system is developed and tested in a Gazebo simulation environment. Key contributions include fine-tuning navigation parameters, optimizing obstacle avoidance, and programmatic control via the Commander API for fully autonomous robot behavior. This work demonstrates ROS 2’s practical application in mobile robot autonomous navigation.

---

## I. Introduction

### A. Robot Operating System (ROS)

ROS is a flexible framework for developing robotic applications. It provides tools, libraries, and conventions that simplify complex robotics software design. ROS supports modular and scalable development with a publisher-subscriber communication model.

The transition from ROS 1 to ROS 2 introduced key improvements such as decentralized communication via DDS, enhanced real-time capabilities, security features, and native multi-robot support.

| Feature                 | ROS 1                 | ROS 2                      |
|-------------------------|-----------------------|----------------------------|
| Communication           | Single master         | Decentralized DDS          |
| Security                | None                  | TLS/DTLS + Authentication  |
| Real-Time               | Not supported         | QoS Policies               |
| Multi-Robot             | Complex setup         | Native support             |
| Discovery               | Centralized           | Distributed                |
| Lifecycle Package Build | Static nodes (catkin) | Managed states (ament/colcon) |

### B. Mobile Robots

Mobile robots use autonomous navigation to safely traverse their environment by processing sensor data for perception, localization, mapping, and path planning. Applications include warehouse automation, service robotics, and search & rescue. Sensors commonly include LiDAR and cameras.

### C. Mapping and Localization

Mapping builds an environmental model; localization estimates the robot’s position within it. Accurate mapping and localization are vital for autonomous operation in dynamic environments.

ROS 2’s Nav2 stack integrates mapping, localization, and path planning into a unified framework supporting modern SLAM algorithms like Cartographer, with enhanced configurability and real-time performance.

### D. Cartographer Algorithm

Google’s Cartographer is a widely used SLAM solution that enables robots to build 2D/3D maps and localize themselves simultaneously. It fuses LiDAR, IMU, and odometry data using submaps and pose graph optimization, ideal for indoor GPS-denied environments.

### E. Problem Statement

Autonomous navigation in GPS-denied, dynamic environments challenges localization accuracy and real-time responsiveness. TurtleBot 3’s operation in narrow, cluttered indoor spaces requires careful tuning of Nav2 and Cartographer parameters to overcome odometry drift and sensor noise, motivating this project’s exploration of ROS 2 capabilities.

---

## II. ROS 2 Simulation and Visualization Setup

### A. Setting Up ROS 2 Environment

- Install ROS 2 Humble on Ubuntu.
- Setup environment variables and build workspace using `colcon`.
- Verify installation with `ros2 topic list`.

### B. Setting Up Gazebo for Simulation

- Install Gazebo ROS packages.
- Launch Gazebo with TurtleBot 3 simulation using URDF/SDF models.
- Use `robot_state_publisher` for broadcasting robot states.

### C. Setting Up RViz for Visualization

- Install RViz2.
- Visualize sensor data, robot pose, occupancy grid, and navigation goals.
- Use 2D Pose Estimate tool and Nav2 Goal for localization and autonomous navigation control.

---

## III. TurtleBot 3 Setup and Mapping

### A. Installing & Configuring TurtleBot 3 Packages

- Install TurtleBot 3 packages including `turtlebot3_description`.
- Set environment variable for TurtleBot 3 model (`waffle`).
- Verify ROS 2 DDS communication topics such as `/cmd_vel`, `/scan`, and `/odom`.

### B. Creating a Basic Map

- Use Cartographer SLAM to generate 2D occupancy grid maps from `/scan` and `/odom`.
- Control robot manually via `teleop_twist_keyboard` to explore environment.
- Save generated map as `.pgm` and `.yaml` files for future use.

### C. Additional ROS 2 Humble Packages

Essential packages include:  
`nav2_costmap_2d`, `nav2_core`, `nav2_behaviors`, `robot_localization`, `gazebo_ros_pkgs`, `turtlebot3_msgs`, `nav2_map_server`, `nav2_bringup`.

These packages support localization (AMCL), navigation planners, costmaps, and behavior trees.

---

## IV. Implementation and Execution of Nav2

### A. Mapping with Cartographer

- Launch `cartographer_node` and `cartographer_occupancy_grid_node` with TurtleBot 3.
- Process LiDAR and odometry to build real-time occupancy maps.

### B. Nav2 Stack Integration

- Launch Gazebo simulation with TurtleBot 3 world.
- Start Nav2 bringup with preconfigured map and navigation parameters.
- Visualize navigation and localization with RViz.

Key tuned parameters include:  
- `inflation_radius`: 0.3 m (for tighter obstacle avoidance)  
- `local_costmap/resolution`: 0.05 m  
- `planner_frequency`: 1.0 Hz  

### C. Manual Mapping

- Control TurtleBot 3 manually with keyboard teleoperation.
- Collect laser scans and odometry data.
- Visualize mapping progress in RViz and save maps.

### D. Single Goal Navigation

- Use `nav2_simple_commander` API for programmatic goal setting.
- Send goals as `PoseStamped` messages to Nav2 action servers.
- Monitor navigation status with timeouts and handle success/failure states.

### E. Multi-Way Points

- Use `BasicNavigator` class to follow multiple waypoints cyclically.
- Implement feedback monitoring and timeout watchdogs.
- Support route inversion for bidirectional patrol.
- Handle Nav2 task results robustly for fault tolerance.

### F. Active Components and Nodes

Key nodes and topics include:

| Node/Topic                | Purpose                          |
|---------------------------|---------------------------------|
| `/amcl`                   | Localization                    |
| `/planner_server`          | Path Planning                  |
| `/controller_server`       | Path Execution                 |
| `/bt_navigator`           | Behavior Tree for tasks        |
| `/turtlebot3_laserscan`   | Laser scan sensor data         |
| `/turtlebot3_imu`         | IMU sensor data                |
| `/camera_driver`          | Camera data                    |
| `/map_server`             | Map and costmap management     |
| `/global_costmap`         | Obstacle avoidance             |
| `/rviz2_node`             | Visualization                  |

Lifecycle managers control node activation and deactivation to ensure robustness.

---

## V. Conclusion

This project successfully developed an autonomous navigation system on TurtleBot 3 using ROS 2 Humble with Cartographer SLAM, AMCL localization, and Nav2 navigation stack. Simulation in Gazebo combined with behavior tree control and costmap tuning enabled effective autonomous navigation and obstacle avoidance.

ROS 2’s modularity and real-time capabilities significantly improve over ROS 1. Future work includes testing on physical robots, integrating 3D LiDAR sensors, applying AI for advanced navigation, and scaling to multi-robot coordination.
