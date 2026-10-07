# Jetson Scripts

This folder contains the scripts to put on the Jetson for ZED-X acquisition. They are run over SSH by `sensors/zedx.py` and should not be modified.

## Files

 - **`zedx_acquisition.py`**: Captures an RGB image and a depth map (uint16, in millimeters) from the left camera of the ZED-X, or saves the camera intrinsics with `--save-intrinsics-only`.

## Setup

1. Install the ZED SDK and its Python API (`pyzed`) on the Jetson, along with `opencv-python` and `numpy`.
2. Copy `zedx_acquisition.py` into the home directory of the Jetson user, which is where `sensors/zedx.py` runs it from.
