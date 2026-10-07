class JointDetectionModel:
    """
    Placeholder for the welding joint detection on the merged Gocator point cloud.

    The vision algorithm is not part of this repository. Replace `process()` with your own
    implementation to use `scripts/demo_pipeline.py`.

    After `process()`, `intersection_data` must be a list of dictionaries, one per detected
    joint, each holding a `line_set` key (`open3d.geometry.LineSet`) whose two points are the
    start and end of the joint, in millimeters, expressed in the robot base frame.
    """
    def __init__(self, ply_file, yaml_config_file=None):
        """
        Args:
            ply_file (str): Path to the merged point cloud (see `vision.gocator.pcd_merger`).
            yaml_config_file (str): Optional path to a configuration file for the algorithm.
        """
        self.ply_file = ply_file
        self.yaml_config_file = yaml_config_file
        self.intersection_data = None

    def process(self, filter_intersections=False, visualize_results=True):
        """
        Detect the welding joints in the point cloud and fill `intersection_data`.

        Args:
            filter_intersections (bool): Keep only one joint when several are found.
            visualize_results (bool): Display the detection results.
        """
        raise NotImplementedError(
            "JointDetectionModel is a placeholder, plug in your own vision algorithm "
            "in vision/gocator/joint_detection.py."
        )
