# Sensors

This module contains sensor interfaces and implementations for robot control systems.

## Files

- **`build_gocator_cffi.py`**: CFFI build script holding a low-level C wrapper around the GoSdk (Gocator 3D scanner SDK), with functions to connect to a Gocator by IP address, trigger scans, receive point cloud data and intensity images, and manage the sensor lifecycle. Running it compiles the wrapper into the `_gocator_wrapper` Python extension module, linked against the GoSdk libraries found in `go-sdk/` (see the main README).

- **`gocator.py`**: High-level Python class (`GocatorCamera`) for interfacing with Gocator 3D laser scanners, built on the `_gocator_wrapper` extension. Provides methods for connecting, triggering scans, retrieving point cloud data, and saving scan data to text or PLY files. Automatically converts from left-handed to right-handed coordinate systems.

- **`zedx.py`**: Python wrapper class (`ZedXCamera`) for interfacing with ZED-X cameras connected to Jetson devices. Manages SSH/SCP connections to remote Jetson hardware for acquiring RGB-D vision data.
