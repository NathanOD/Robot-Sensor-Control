import json
import numpy as np
import open3d as o3d
from pathlib import Path

class PointCloudMerger:
    def __init__(
        self,
        data_dir: Path = Path("data"),
        hand_eye_file: Path = Path("hand_eye_matrix.json"),
        output_ply: Path | None = None,
    ) -> None:
        self.data_dir = Path(data_dir)
        self.poses_dir = self.data_dir / "poses"
        self.scans_dir = self.data_dir / "scans"
        self.hand_eye_file = Path(hand_eye_file)
        self.output_ply = output_ply or (self.data_dir / "reprojected.ply")

    @staticmethod
    def read_pose_matrix(filepath: Path) -> np.ndarray:
        with open(filepath, "r") as file_handle:
            text = file_handle.read()
        text = text.replace("[", "").replace("]", "").replace(";", "").replace(",", " ")
        return np.fromstring(text, sep=" ").reshape(4, 4)

    @staticmethod
    def load_hand_eye_matrix(filepath: Path) -> np.ndarray:
        if not filepath.exists():
            raise FileNotFoundError(f"Hand-eye file not found: {filepath}")

        with open(filepath, "r") as file_handle:
            payload = json.load(file_handle)

        if "matrix" in payload:
            matrix = payload["matrix"]
        elif "H_cam2gripper" in payload:
            matrix = payload["H_cam2gripper"]
        else:
            raise KeyError("Expected a top-level 'matrix' key in hand-eye JSON.")

        hand_eye = np.asarray(matrix, dtype=float)
        if hand_eye.shape != (4, 4):
            raise ValueError(f"Hand-eye matrix must be 4x4, got {hand_eye.shape}.")
        return hand_eye

    @staticmethod
    def load_point_cloud(path: Path) -> o3d.geometry.PointCloud:
        point_cloud = o3d.io.read_point_cloud(str(path))
        if point_cloud.is_empty():
            raise ValueError(f"Empty point cloud: {path}")
        return point_cloud

    def process(self) -> Path:
        hand_eye = self.load_hand_eye_matrix(self.hand_eye_file)

        scan_files = sorted(self.scans_dir.glob("pose_*.ply"))
        if not scan_files:
            raise FileNotFoundError(f"No PLY scans found in {self.scans_dir}")

        merged = o3d.geometry.PointCloud()

        for scan_path in scan_files:
            pose_path = self.poses_dir / f"{scan_path.stem}.txt"
            if not pose_path.exists():
                print(f"Skipping {scan_path.name}: missing pose file {pose_path.name}")
                continue

            scan_cloud = self.load_point_cloud(scan_path)
            pose_mat = self.read_pose_matrix(pose_path)

            transform = pose_mat @ hand_eye
            scan_cloud.transform(transform)
            merged += scan_cloud

            #print(f"Processed {scan_path.name}")

        if merged.is_empty():
            raise RuntimeError("No scans were merged. Check pose and scan filenames.")

        o3d.io.write_point_cloud(str(self.output_ply), merged)
        print(f"Successfully merged {len(scan_files)} scans into {self.output_ply}")
        return self.output_ply