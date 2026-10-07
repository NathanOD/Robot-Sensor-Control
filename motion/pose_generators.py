import os
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

def torch_angle_from_line(start_point, end_point):
    """
    Compute the torch angle (rx) for a linear welding path defined by start and end points.
    
    Args:
        start_point (list or array): The starting point of the line (x, y, z).
        end_point (list or array): The ending point of the line (x, y, z).
    
    Returns:
        float: The computed torch angle (rx) in radians, which is the angle between the line and the horizontal plane.
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    
    rz_angle = math.atan2(dy, dx)

    #if rz_angle < 0:
    #    rz_angle += math.pi
    
    return rz_angle

class ZedxPoseGenerator:
    """
    Generate poses for the ZedX camera based on a table configuration and a fixed viewing angle.
    """
    def __init__(self, config_dir="config", distance_from_center=0.8, angle=math.pi/8.0):
        config_path = os.path.join(config_dir, "table_config.yaml")
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)
        
        self.width = self.config["width"]
        self.height = self.config["height"]
        self.cx = self.width / 2
        self.cy = self.height / 2
        self.distance_from_center = distance_from_center
        self.angle = angle

    def get_angles(self, x, y, z):
        """
        Compute the rotation angles (rx, ry, rz) for the camera to look at the center of the table from a given position.
        """
        ry = math.atan2(x - self.cx, z)
        rx = math.atan2(self.cy - y, z)
        return rx, ry, 0.0

    def generate_poses(self):
        """
        Generate a list of poses for the ZedX camera around the table.
        """
        z_angled = self.distance_from_center * math.cos(self.angle)
        r = self.distance_from_center * math.sin(self.angle)

        positions = [
            {"id": 1, "name": "Center", "x": self.cx, "y": self.cy, "z": self.distance_from_center},
            {"id": 2, "name": "Left", "x": self.cx - r, "y": self.cy, "z": z_angled},
            {"id": 3, "name": "Right", "x": self.cx + r, "y": self.cy, "z": z_angled},
            {"id": 4, "name": "Top", "x": self.cx, "y": self.cy + r, "z": z_angled},
            {"id": 5, "name": "Bottom", "x": self.cx, "y": self.cy - r, "z": z_angled}
        ]
        
        poses = []
        for pos in positions:
            rx, ry, rz = self.get_angles(pos["x"], pos["y"], pos["z"])
            poses.append({
                "id": pos["id"], "name": pos["name"],
                "x": pos["x"], "y": pos["y"], "z": pos["z"],
                "rx": rx, "ry": ry, "rz": rz
            })
        
        return poses


class GocatorPoseGenerator:
    """
    Produce scan poses for a Gocator along a linear trajectory.
    """
    def __init__(self, start_point: list, end_point: list, fov_width: float = 0.154, overlap: float = 0.0, ry: float = -math.pi / 4.0):
        self.start_point = np.asarray(start_point, dtype=float)
        self.end_point = np.asarray(end_point, dtype=float)
        self.fov_width = float(fov_width)
        self.overlap = float(overlap)
        self.step = self.fov_width * (1.0 - self.overlap)
        self.ry = float(ry)

    def generate_line_points(self):
        """
        Compute a list of points along a line from start to end, spaced by the effective FOV width.
        Superfluous poses at the end are removed if the final area is already covered.
        
        Returns:
            np.ndarray: An Nx3 array of XYZ points along the line from start to end.
        """
        if self.step <= 0:
            raise ValueError("effective step must be strictly positive")

        direction = self.end_point - self.start_point
        distance = float(np.linalg.norm(direction))

        direction_unit = direction / distance
        
        # Calculate how many full steps we can take
        count = int(np.floor(distance / self.step)) + 1
        distances = np.arange(count, dtype=float) * self.step
        
        # Check if the last point's coverage already includes the end point
        # A scan covers [-fov_width/2, fov_width/2] centered at the point (assuming FOV is along the line)
        points = self.start_point + np.outer(distances, direction_unit)

        # If the end_point is not fully covered by the last pose, add a pose for the end_point
        # Usually, if distances[-1] + self.fov_width/2 < distance, we might need one more, 
        # but if we just want to remove superfluous ones at the end that the original code added blindly:
        if distances[-1] + self.fov_width / 2 < distance - 1e-6:
            points = np.vstack((points, self.end_point))

        return points

    @staticmethod
    def get_rx_slope(start_point, end_point):
        """
        Compute the slope `rx` angle (pitch) for a 3D line.
        """
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        dz = end_point[2] - start_point[2]
        return math.atan2(dz, math.hypot(dx, dy))

    @staticmethod
    def get_rz(start_point, end_point):
        """
        Compute `rz` so the scanner (or end-effector) faces along the path.
        
        Args:
            start_point (list or array): The starting point of the line (x, y, z).
            end_point (list or array): The ending point of the line (x, y, z).
            
        Returns:
            float: The `rz` angle in radians to face along the path from start to end.
        """
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        rz = math.atan2(dy, dx) - (math.pi / 2.0)
        if rz < -math.pi:
            rz += math.pi
        return rz

    def generate_poses(self):
        """
        Compute full poses (position + orientation) for the Gocator along the line from start to end.
        
        Returns:
            list: A list of pose dictionaries, each containing 'x', 'y', 'z', 'rx', 'ry', and 'rz' keys.
        """
        pts = self.generate_line_points()
        ry_val = self.ry
        rz_val = self.get_rz(self.start_point, self.end_point)
        rx_slope = self.get_rx_slope(self.start_point, self.end_point)
        print(rz_val)
        # Convert the Euler sequence (intrinsic Y, then intrinsic X, then intrinsic Z) to a rotation vector.
        rot = R.from_euler('yxz', [ry_val, rx_slope, rz_val], degrees=False)
        rotvec = rot.as_rotvec()

        poses = []
        for p in pts:
            poses.append({
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
                "rx": float(rotvec[0]),
                "ry": float(rotvec[1]),
                "rz": float(rotvec[2]),
                "euler_rz": float(rz_val)
            })
        return poses
