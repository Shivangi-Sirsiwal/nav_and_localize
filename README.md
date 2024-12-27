# Robot Navigation and Mapping Project

A ROS-based implementation for autonomous robot navigation, mapping, and localization using TurtleBot3.

## Project Description

This project implements autonomous navigation and mapping capabilities for a TurtleBot3 robot in an unknown maze environment. The robot accomplishes three main tasks:

1. Autonomously explores and maps an unknown maze using frontier-based exploration
2. Returns to its starting position after mapping
3. Localizes itself when placed at a random point in the maze and navigates to a specified goal

The repository includes code for both physical robot operation and simulation:
- Gazebo simulation for testing navigation algorithms
- RViz visualization for monitoring robot state, sensor data, and map building
- Core navigation and mapping algorithms for TurtleBot3 control

## Features

### Mapping
- Utilizes ROS G-Mapping node for occupancy grid generation
- Implements C-space gradient for obstacle inflation
- Configurable map resolution and size parameters

### Path Planning & Following
- A* algorithm implementation for optimal path planning
- Proportional control for path following
- Drive-forward-turn-drive-forward system with heading error threshold
- Path centering between obstacles using C-Space gradient

### Frontier Exploration
- Automated frontier detection using occupancy grid analysis
- Frontier cell grouping via wavefront algorithm
- Interest calculation based on:
  - Frontier size (weighted x5)
  - Distance to frontier centroid

### Localization
- AMCL implementation for pose estimation
- Covariance-based position refinement
- Real-world pose transformation for accurate navigation

## Implementation Results

### G-Mapping Results
<img src="https://github.com/user-attachments/assets/5b381e42-cf42-40be-98c3-b0a6a5a4f25f" width = 400>

*Figure 1: The completed G-Mapping map of the maze*

### Path Planning Visualization
<img src="https://github.com/user-attachments/assets/4b3162b5-4cc0-442b-b943-a15b41209d5d" width = 800>

*Figures 2 & 3: The A-Star path, C-Space, and C-Space gradient with the robot path following*

### AMCL Localization
<img src ="https://github.com/user-attachments/assets/5de41d64-344a-4df1-8f13-ca6803f40911" width = 600>

*Figure 4: The robot localization using AMCL*

## Implementation Details

### Phase 1 & 2: Mapping and Navigation
- G-Mapping converts LiDAR data to occupancy grid
- C-space node handles obstacle inflation and gradient generation
- Path truncation during mapping for regular map updates

### Phase 3: Localization
- AMCL package for Monte Carlo localization
- In-place rotation until covariance threshold met
- Pose publishing for navigation execution

## Challenges and Solutions

1. **Path Following:**
   - Issue: Initial A* paths incompatible with robot diameter
   - Solution: Tuned A* heuristics and C-Space for centered paths

2. **Curve Navigation:**
   - Issue: Inaccurate mapping of unknown curves
   - Solution: Implemented halfway-point navigation strategy

3. **Frontier Exploration:**
   - Issue: Frontier oscillation
   - Solution: Balanced exploration vs. safety tradeoff

## Technologies Used
- ROS
- G-Mapping
- AMCL
- TurtleBot3
