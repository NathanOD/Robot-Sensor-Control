# Robot Control (UR10e)

Python code to control a UR10e robot arm (through RTDE) and the sensors mounted on it:

- a **ZED-X** stereo camera, driven through a Jetson over SSH, for RGB-D captures around a table;
- a **Gocator** 3D laser scanner, controlled from Python through a wrapper of the LMI GoSdk (C SDK), for high-resolution scans along a line;
- a **welding torch**, moved along a line expressed in the table frame.

The vision algorithms used in the full pipeline (coarse welding line prediction from the ZED-X
captures and joint detection on the Gocator point cloud) are **not included**. They are replaced by
placeholders, see [Vision placeholders](#vision-placeholders).

## Project structure

```text
.
├── config/          YAML/JSON configuration (network, TCPs, table, home position, hand-eye)
├── core/            Robot connection (RobotController), forward kinematics and frame transforms
├── motion/          Pose generators (ZED-X viewpoints, Gocator scan line, torch angle)
├── sensors/         Sensor interfaces (GocatorCamera, ZedXCamera) and the Python wrapper of the Gocator SDK
├── jetson_scripts/  Acquisition script to copy on the Jetson driving the ZED-X
├── vision/          Point cloud merging and placeholders for the vision algorithms
└── scripts/         Executable scripts
```

## Installation

### Python environment

```shell
conda create -n robot_control python=3.12
conda activate robot_control
pip install -r requirements.txt
```

The ZED-X interface also needs `sshpass` on the host machine:

```shell
sudo apt install sshpass
```

### Gocator SDK and Python wrapper

The Gocator is controlled from Python through a wrapper of the LMI GoSdk, which is a C library.
`sensors/build_gocator_cffi.py` holds a small C layer over the SDK (connection, trigger, reception of
the point cloud and of the intensity image) and compiles it with [CFFI](https://cffi.readthedocs.io)
into the `sensors._gocator_wrapper` Python extension, linked against the SDK libraries.
`GocatorCamera` (`sensors/gocator.py`) uses this extension and exposes the sensor as a Python class:

```text
GocatorCamera                  sensors/gocator.py
└── _gocator_wrapper           CFFI extension built by sensors/build_gocator_cffi.py
    └── GoSdk + kApi           go-sdk/lib/linux_x64d/libGoSdk.so, libkApi.so
        └── Gocator sensor     Ethernet, IP set in config/network_config.yaml
```

The SDK is not distributed with this repository, so it must be downloaded and the wrapper built locally:

1. Download the **Gocator SDK 6.5.2.5** from the LMI website:
   https://lmi3d.com/resource/gocator-sdk-version-6-5-2-5/
2. Extract it at the root of the project and rename the extracted `GO_SDK` folder to `go-sdk`
   (lowercase):

   ```shell
   unzip 14400-6.5.2.5_SOFTWARE_GO_SDK.zip
   mv GO_SDK go-sdk
   ```

3. Build the Linux libraries of the SDK (requires `gcc` and `make`):

   ```shell
   cd go-sdk/Gocator
   make -f GoSdk-Linux_X64.mk
   cd ../..
   ```

   This creates `libGoSdk.so` and `libkApi.so` in `go-sdk/lib/linux_x64d/`.

4. Compile the Python wrapper around the SDK:

   ```shell
   cd sensors
   python build_gocator_cffi.py
   cd ..
   ```

   This creates the `_gocator_wrapper` extension (`.so`) in `sensors/`. Check that it loads:

   ```shell
   python -c "from sensors.gocator import lib; print('Gocator wrapper loaded:', lib is not None)"
   ```

   If the wrapper is not built, importing `sensors.gocator` prints a warning and `take_scan()`
   returns an error status.

The scanner can then be used from Python:

```python
from sensors.gocator import GocatorCamera

scanner = GocatorCamera(config_dir="config")
result = scanner.take_scan()  # Nx3 point cloud in the scanner frame, in millimeters
if result["status"] == "success":
    scanner.save_scan(result["data"], pose_num=0, save_dir="data_gocator/scans", format="ply")
```

### Jetson (ZED-X)

The ZED-X is plugged into a Jetson, which the host drives over SSH.

1. Install the ZED SDK and its Python API (`pyzed`) on the Jetson, along with `opencv-python` and `numpy`.
2. Copy `jetson_scripts/zedx_acquisition.py` into the home directory of the Jetson user.
3. Fill in the Jetson address and credentials in `config/jetson_config.yaml`.

## Configuration

Everything is configured in the `config/` folder:

- **`network_config.yaml`**: IP addresses of the robot, the Jetson and the Gocator.
- **`jetson_config.yaml`**: SSH access to the Jetson (host, username, password, port).
- **`tcp_config.yaml`**: TCP offsets of each tool (`zedx`, `gocator`, `torch`), as `x, y, z` in meters and a rotation vector `rx, ry, rz` in radians.
- **`table_config.yaml`**: table dimensions (`width` along X, `height` along Y, in meters) and pose of the table frame in the robot base frame (origin `delta_x, delta_y, delta_z` and rotation vector `rot_x, rot_y, rot_z`).
- **`home_config.yaml`**: joint angles of the Home position.
- **`gocator_handeye.json`**: hand-eye calibration of the Gocator (4x4 matrix, in millimeters, from the scanner to the flange).

The DH calibration corrections in `core/pose_utils.py` are specific to our UR10e. Replace them with
the values of your robot, or call `get_flange_pose_matrix(q, use_calibration=False)` to use the
nominal UR10e parameters.

## Usage

Always run the scripts from the root of the project so that the Python modules and the `config/`
folder are found.

| Script | Description |
| --- | --- |
| `scripts/go_home.py` | Move the robot to the Home position defined in `config/home_config.yaml`. |
| `scripts/acq_zedx.py` | Single ZED-X capture at the current robot position. |
| `scripts/acq_gocator.py` | Single Gocator scan at the current robot position. |
| `scripts/demo_zedx.py` | Capture sequence around the table with the ZED-X (5 viewpoints). |
| `scripts/demo_gocator.py` | Scan sequence along a line of the table with the Gocator. |
| `scripts/demo_torch.py` | Move the torch along a welding line given in the table frame. |
| `scripts/demo_pipeline.py` | Full pipeline: ZED-X sequence, coarse line prediction, Gocator scans along that line, point cloud merging, joint detection and torch motion. Requires the vision placeholders to be implemented. |

```shell
python scripts/go_home.py
python scripts/demo_zedx.py
python scripts/demo_gocator.py
```

### ZED-X viewpoints

`ZedxPoseGenerator` (in `motion/pose_generators.py`) generates 5 viewpoints, all looking at the
center of the table: one above the center (`Center`) and four tilted by `angle` on each side
(`Left`, `Right`, `Top`, `Bottom`), all at `distance_from_center` from the center of the table.

### Output data

The acquisitions are written to `data_zedx/` and `data_gocator/` (deleted at the start of each run):

```text
data_zedx/
├── poses/pose_XX.txt        flange pose of each viewpoint
└── captures/
    ├── image_XX.png         RGB image (left camera)
    └── depth_XX.png         depth map (uint16, in millimeters)

data_gocator/
├── poses/pose_XX.txt        flange pose of each scan
└── scans/pose_XX.ply        point cloud in the scanner frame (in millimeters)
```

Pose files hold the 4x4 flange pose matrix in the robot base frame, with translations in millimeters.

## Vision placeholders

The following modules only define the interface expected by `scripts/demo_pipeline.py` and raise
`NotImplementedError`:

- **`vision/zedx/coarse_pred.py`**: `CoarseLinePredictor.run()` must return the start and end points
  of the welding line (2x3 array, in meters, in the robot base frame) from the ZED-X captures, or `None`.
- **`vision/gocator/joint_detection.py`**: `JointDetectionModel.process()` must fill `intersection_data`
  with the detected joints, from the merged Gocator point cloud.

`vision/gocator/pcd_merger.py` merges the Gocator scans in the robot base frame using the flange
poses and the hand-eye calibration.

## Troubleshooting

If the Gocator connection drops or the scans fail, disable the energy saving and offloading features
of the network interface connected to the scanner (replace `<interface>` with its name, e.g. `enp11s0`):

```shell
sudo ip link set <interface> down && sudo ip link set <interface> up
sudo ethtool --set-eee <interface> eee off
sudo ethtool -K <interface> tso off gso off gro off
```
