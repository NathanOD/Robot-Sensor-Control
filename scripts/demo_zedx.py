import math
import os
import sys
import shutil
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.zedx import ZedXCamera
from core.robot import RobotController
from motion.pose_generators import ZedxPoseGenerator
from core.pose_utils import get_flange_pose_matrix, save_pose_matrix

OUTPUT_DIR = "data_zedx"
DISTANCE_FROM_CENTER = 0.45
ANGLE = math.pi/4.0

def main():

    print("ZED-X CAMERA SEQUENCE")

    # Init robot, camera, and pose generator
    robot = RobotController(config_dir="config")
    camera = ZedXCamera(config_dir="config")
    generator = ZedxPoseGenerator(config_dir="config",
                                  distance_from_center=DISTANCE_FROM_CENTER,
                                  angle=ANGLE)
    
    # Create output directory
    output_path = Path(OUTPUT_DIR)
    poses_path = output_path / "poses"
    if output_path.exists():
        shutil.rmtree(output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    poses_path.mkdir(parents=True, exist_ok=True)

    #camera.acquire_intrinsics(output_dir=OUTPUT_DIR)
    poses = generator.generate_poses()

    robot.set_active_tcp("zedx")

    for i, pose in enumerate(poses):
        print(f"Moving to pose: {pose['name']}")
        robot.move_to_pose(
            x=pose['x'], 
            y=pose['y'], 
            z=pose['z'],
            rx=pose['rx'], 
            ry=pose['ry'], 
            rz=pose['rz'],
            frame="table"
        )

        # Retrieve and save the flange pose matrix
        flange_matrix = get_flange_pose_matrix(robot.get_actual_q())
        save_pose_matrix(flange_matrix, pose_num=i, save_dir=str(poses_path), file_prefix="pose_")

        # Capture RGB-D data
        camera.capture_rgbd()

    camera.download_captures(output_dir=OUTPUT_DIR)

    print("Sequence completed. Data saved to:", OUTPUT_DIR)

    # Go back to home position
    robot.go_home()

if __name__ == "__main__":
    main()