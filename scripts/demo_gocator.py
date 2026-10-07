import os
import sys
import shutil
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.robot import RobotController
from sensors.gocator import GocatorCamera
from motion.pose_generators import GocatorPoseGenerator
from core.pose_utils import get_flange_pose_matrix, save_pose_matrix

OUTPUT_DIR = "data_gocator"
FOV_WIDTH = 0.12
COARSE_LINES = [
    ([0.10, 0.14, 0.0], [0.41, 0.14, 0.0]),
    #([0.51, 0.31, 0.0], [0.0, 0.0, 0.0]),
    #([0.0, 0.0, 0.0], [0.51, 0.31, 0.0])
    #([0.10, 0.14, 0.0], [0.41, 0.14, 0.0])
]

def main():
    
    print("GOCATOR CAMERA SEQUENCE")

    # Init robot and camera
    robot = RobotController(config_dir="config")
    scanner = GocatorCamera(config_dir="config")

    # Create output directory
    output_path = Path(OUTPUT_DIR)
    poses_path = output_path / "poses"
    scans_path = output_path / "scans"
    if output_path.exists():
        shutil.rmtree(output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    poses_path.mkdir(parents=True, exist_ok=True)
    scans_path.mkdir(parents=True, exist_ok=True)

    # Move to the first pose (approach position) to prevent collision
    robot.set_active_tcp("zedx")
    robot.go_to_table_center(z=0.6, rz=0.0)
    robot.set_active_tcp("gocator")
    robot.go_to_table_center(z=0.6, rz=-3.14159/2.0)

    global_pose_idx = 0

    for line_idx, line in enumerate(COARSE_LINES):
        print(f"Processing line {line_idx}: {line}")
        generator = GocatorPoseGenerator(start_point=line[0],
                                         end_point=line[1],
                                         fov_width=FOV_WIDTH,
                                         overlap=0.1)

        poses = generator.generate_poses()

        # Move to the first pose (approach position) to prevent collision
        robot.go_to_table_center(z=0.8, rz=poses[0]['euler_rz'])

        for i, pose in enumerate(poses):
            print(f"Moving to pose {global_pose_idx}")
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
            save_pose_matrix(flange_matrix, pose_num=global_pose_idx, save_dir=str(poses_path), file_prefix="pose_")

            # Capture scan data
            result = scanner.take_scan()
            if result["status"] == "success":
                scanner.save_scan(result["data"], pose_num=global_pose_idx, save_dir=str(scans_path), file_prefix="pose_", format="ply")
            else:
                print(f"Scan failed at pose {global_pose_idx}.")
            
            global_pose_idx += 1
    
    print("Sequence completed. Data saved to:", OUTPUT_DIR)


if __name__ == "__main__":
    main()