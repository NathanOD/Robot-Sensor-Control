# Vision

This module contains the processing applied to the sensor data in `scripts/demo_pipeline.py`.

The vision algorithms are not part of this repository: `coarse_pred.py` and `joint_detection.py` are placeholders that define the interface expected by the pipeline and raise `NotImplementedError`.

## Files

 - **`zedx/coarse_pred.py`**: Placeholder (`CoarseLinePredictor`) for the coarse welding line prediction from the ZED-X captures. `run()` must return the start and end points of the line (2x3 array, in meters, in the robot base frame), or `None` if no line is found.

 - **`gocator/pcd_merger.py`**: Merges the Gocator scans into a single point cloud in the robot base frame, using the flange pose of each scan and the hand-eye calibration (`config/gocator_handeye.json`).

 - **`gocator/joint_detection.py`**: Placeholder (`JointDetectionModel`) for the welding joint detection on the merged Gocator point cloud. `process()` must fill `intersection_data` with one dictionary per joint, holding a `line_set` (`open3d.geometry.LineSet`) whose two points are the start and end of the joint, in millimeters, in the robot base frame.
