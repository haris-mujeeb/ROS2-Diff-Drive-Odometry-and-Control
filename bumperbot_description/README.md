# Bumperbot Description

This package contains all the necessary files to describe the Bumperbot robot within the ROS 2 framework. It is a core component of the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial, providing the robot's physical and visual definition for simulation and visualization.

## Concepts Covered (Notes)

*   **URDF/XACRO:** Understanding the Unified Robot Description Format (URDF) and its extension XACRO for defining robot kinematics, dynamics, and visual properties.
*   **Gazebo Integration:** Adding Gazebo-specific properties to the URDF for simulation, including physics, sensors, and plugins.
*   **Meshes:** Using 3D model files (e.g., `.STL`) for accurate visual representation of robot components.
*   **Launch Files:** Creating Python launch files to spawn the robot in Gazebo and configure `robot_state_publisher`.
*   **TF (Transformations):** How `robot_state_publisher` broadcasts the robot's joint states as transformations to the TF tree.

## Key Files and Directories

*   `urdf/`: Contains the `bumperbot.urdf.xacro` (robot definition) and `bumperbot.gazebo.xacro` (Gazebo extensions).
*   `meshes/`: Stores 3D model files (e.g., `.STL`) for robot parts.
*   `launch/`: Includes launch files like `gazebo.launch.py` for starting the simulation.
*   `rviz/`: Contains Rviz configuration files (e.g., `bumperbot.rviz`).

## How to Build and Run

To build this package:
```bash
colcon build --packages-select bumperbot_description
```

To launch the Bumperbot simulation in Gazebo:
```bash
ros2 launch bumperbot_description gazebo.launch.py
```

This command will:
*   Start the `robot_state_publisher` to make the robot's URDF available.
*   Set up the necessary environment variables for Gazebo.
*   Launch the Gazebo simulator.
*   Spawn the Bumperbot model into the Gazebo world using the URDF defined in `urdf/bumperbot.urdf.xacro` and its Gazebo extensions.
