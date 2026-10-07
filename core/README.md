# Core

This module contains robot interface and spatial transformation implementations for robot control systems.

## Files

 - **`pose_utils.py`**: Forward kinematics of the UR10e (DH parameters with optional calibration corrections) to compute the flange pose from joint angles, and export of pose matrices to text files.

 - **`robot.py`**: High-level robot control API. Provides the `RobotController` class, which connects to the robot through RTDE, selects the active TCP, moves the robot to the Home position or to poses expressed in the table or base frame, and reads the current joint angles and TCP pose.

 - **`transform.py`**: Spatial transform utilities used across the project. Contains the DH transformation matrix, conversions between rotation vectors and rotation matrices, the table to base frame transformation, and helpers to apply transforms to points, vectors and detection results.
