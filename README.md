# Wall-Following Robot Project

This project aims to create a robot that autonomously follows a wall using ROS 2, implementing both topic-based control for navigation and services/actions for added functionalities. The project has been tested and validated in a simulated environment before being deployed to a real robot.

---

## Table of Contents
- [Overview](#overview)
- [Part I: Wall-Following Behavior Using Topics](#part-i-wall-following-behavior-using-topics)
- [Part II: Wall Detection Using Services](#part-ii-wall-detection-using-services)
- [Part III: Odometry Recording Using Actions](#part-iii-odometry-recording-using-actions)
- [How to Test](#how-to-test)
- [Project Structure](#project-structure)
---

## Overview
The wall-following behavior enables the robot to maintain a distance of 30 cm (1 foot) from the wall on its right-hand side while navigating around obstacles. The project is divided into three main components:

1. **Wall-following behavior using topics**: Uses laser scans to follow the wall.
2. **Service to find and approach the nearest wall**: Allows the robot to autonomously find a wall before starting its wall-following behavior.
3. **Action to record odometry data**: Tracks and records the robot’s position and distance traveled.

---

## Part I: Wall-Following Behavior Using Topics

### Objective
Create a ROS 2 node to control the robot’s motion based on distance readings from its laser scanner. The robot maintains a fixed distance from the wall on its right-hand side, making adjustments to its trajectory based on the laser readings.

### Implementation Steps
1. **Subscribe to Laser Scan Topic**:  
   Capture data from the `/scan` topic to measure the distance to the wall.
   
2. **Adjust Movement Based on Distance**:  
   - **Distance > 0.3m**: Move closer to the wall.
   - **Distance < 0.2m**: Move away from the wall.
   - **Distance between 0.2m and 0.3m**: Continue moving forward.
   
3. **Handle Corners**:  
   If a wall is detected directly ahead, make a quick left turn to avoid collision.

### Testing
- Test in a simulation environment before deploying to a real robot.
- Use teleop controls to move the robot near a wall before starting the program.

---

## Part II: Wall Detection Using Services

### Objective
Create a service server that, when called, autonomously moves the robot to the nearest wall and aligns it for wall-following.

### Implementation Steps
1. **Create a `find_wall` Service**:  
   A custom service that locates the nearest wall using laser scan data and positions the robot accordingly.
   
2. **Integrate with Wall-Following Node**:  
   Call the `find_wall` service before starting the main wall-following behavior.

3. **Launch with `main.launch.py`**:  
   Create a launch file to start both the wall-finding service and the wall-following node.

### Testing
- Manually call the service to verify wall detection and alignment.
- Test both the wall-finding and wall-following behaviors in sequence.

---

## Part III: Odometry Recording Using Actions

### Objective
Create an action server to record the robot’s odometry data as it moves, providing feedback on the distance traveled.

### Implementation Steps
1. **Create `record_odom` Action Server**:  
   A server that records (x, y, theta) positions as the robot moves, providing feedback on the total distance traveled.

2. **Integrate Action Client with Wall-Following Node**:  
   The wall-following node calls the action server to start odometry recording.

3. **Launch with `main.launch.py`**:  
   Include the action server launch in the primary launch file.

### Testing
- Monitor the feedback from the action server to ensure odometry is recorded accurately.
- Validate that the robot moves and records data correctly throughout the process.

---

## How to Test

1. **Testing in Simulation**:  
   Launch the simulation environment and use `teleop_twist_keyboard` to manually position the robot before starting the ROS program.
   
2. **Run the Nodes**:  
   Launch all nodes using `main.launch.py`. Verify the behavior of wall-following, wall detection, and odometry recording.

3. **Testing on Real Robot**:  
   Once simulation tests are successful, schedule a session in the Real Robot Lab to test on a physical robot.

---

## Project Structure

