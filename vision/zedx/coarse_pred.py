import numpy as np


class CoarseLinePredictor:
    """
    Placeholder for the coarse welding line prediction from the ZED-X acquisitions.

    The vision algorithm is not part of this repository. Replace `run()` with your own
    implementation to use `scripts/demo_pipeline.py`.

    Expected content of `data_dir` (written by `scripts/demo_pipeline.py`):
        - `captures/image_XX.png`: RGB images from the left camera
        - `captures/depth_XX.png`: depth maps (uint16, in millimeters)
        - `poses/pose_XX.txt`: flange pose matrices (see `core.pose_utils.save_pose_matrix`)
        - `intrinsics_zedx.json`: left camera intrinsics (fx, fy, cx, cy)
    """
    def __init__(self, data_dir="data_zedx", seed: int = 1):
        self.data_dir = data_dir
        self.seed = seed

    def run(self) -> np.ndarray | None:
        """
        Predict the coarse welding line from the ZED-X acquisitions.

        Returns:
            numpy.ndarray: A 2x3 array holding the start and end points of the welding line,
                in meters, expressed in the robot base frame. None if no line is found.
        """
        raise NotImplementedError(
            "CoarseLinePredictor is a placeholder, plug in your own vision algorithm "
            "in vision/zedx/coarse_pred.py."
        )
