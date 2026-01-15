# Bumperbot Python Examples

This package contains Python examples for the Bumperbot ROS 2 project. These examples are developed as part of the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial and demonstrate fundamental ROS 2 concepts using Python.

## Concepts Covered (Notes)

*   **Creating a Service Server**: `simple_service_server.py` shows how to create a ROS 2 service using a custom service type (`bumperbot_msgs/srv/AddTwoInts`).
*   **Broadcasting TF Transforms**: `simple_tf_kinematics.py` demonstrates broadcasting both static and dynamic transforms to `/tf` and `/tf_static`, simulating robot movement.
*   **Quality of Service (QoS)**: `simple_qos_publisher.py` is a publisher that can be configured with different QoS profiles for `reliability` (`BEST_EFFORT` or `RELIABLE`) and `durability` (`VOLATILE` or `TRANSIENT_LOCAL`). This is crucial for controlling how messages are handled in different network conditions.
*   **Lifecycle Nodes**: `simple_lifecycle_node.py` demonstrates a managed node. Lifecycle nodes have a state machine (e.g., `unconfigured`, `inactive`, `active`, `finalized`) that can be controlled externally, allowing for more deterministic setup and shutdown of ROS systems.

## Examples

### 1. `simple_service_server.py`

*   **Description**: A node that provides an `add_two_ints` service.
*   **To Run**: `ros2 run bumperbot_py_examples simple_service_server`
*   **To Call**: `ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"`

### 2. `simple_tf_kinematics.py`

*   **Description**: Publishes static and dynamic TF transforms, simulating robot motion.
*   **To Run**: `ros2 run bumperbot_py_examples simple_tf_kinematics`
*   **To Inspect**: `ros2 run tf2_ros tf2_echo odom bumperbot_base`

### 3. `simple_qos_publisher.py`

*   **Description**: A publisher with configurable QoS settings.
*   **To Run**:
    ```bash
    # Example with reliable, transient_local durability
    ros2 run bumperbot_py_examples simple_qos_publisher --ros-args -p reliability:=reliable -p durability:=transient_local
    ```
*   **To Listen**: In another terminal, run `ros2 topic echo /character`. Because of `transient_local`, a late-joining subscriber will still receive the last published message.

### 4. `simple_lifecycle_node.py`

*   **Description**: A node with a managed lifecycle.
*   **To Run**: `ros2 run bumperbot_py_examples simple_lifecycle_node`
*   **To Control the Lifecycle**:
    *   Check state: `ros2 lifecycle get /simple_lifecycle_node`
    *   Configure: `ros2 lifecycle set /simple_lifecycle_node configure`
    *   Activate: `ros2 lifecycle set /simple_lifecycle_node activate`
    *   Publish to the node: `ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello lifecycle'}"`
    *   Deactivate: `ros2 lifecycle set /simple_lifecycle_node deactivate`
    *   Cleanup: `ros2 lifecycle set /simple_lifecycle_node cleanup`

## How to Build

To build this package:
```bash
colcon build --packages-select bumperbot_py_examples
```
