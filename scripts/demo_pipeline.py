import os
import sys
import time
import shutil
import numpy as np
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.zedx import ZedXCamera
from sensors.gocator import GocatorCamera
from core.robot import RobotController
from motion.pose_generators import ZedxPoseGenerator, GocatorPoseGenerator
from vision.zedx.coarse_pred import CoarseLinePredictor
from vision.gocator.pcd_merger import PointCloudMerger
from vision.gocator.joint_detection import JointDetectionModel
from core.pose_utils import get_flange_pose_matrix, save_pose_matrix
from motion.pose_generators import torch_angle_from_line
from core.transform import convert_points_base_to_table

ZEDX_OUTPUT_DIR = "data_zedx"
GOCATOR_OUTPUT_DIR = "data_gocator"
DISTANCE_FROM_CENTER = 0.7
ANGLE = 3.14159 / 7.0
FOV_WIDTH = 0.12
TORCH_OFFSET_Y = 0.002
TORCH_OFFSET_Z = -0.008

def main():

    # Init robot, camera, and pose generator
    robot = RobotController(config_dir="config")
    camera = ZedXCamera(config_dir="config")
    scanner = GocatorCamera(config_dir="config")
    generator = ZedxPoseGenerator(config_dir="config",
                                  distance_from_center=DISTANCE_FROM_CENTER,
                                  angle=ANGLE)
    coarse_predictor = CoarseLinePredictor(data_dir=f"{ZEDX_OUTPUT_DIR}")
    
    robot.go_home(speed=0.5, acc=0.5)
    
    print("ZED-X CAMERA SEQUENCE")
    
    # Create ZEDX output directory
    output_path = Path(ZEDX_OUTPUT_DIR)
    zed_poses_path = output_path / "poses"
    pcd_path = output_path / "ply_outputs"
    if output_path.exists():
        shutil.rmtree(output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    zed_poses_path.mkdir(parents=True, exist_ok=True)
    pcd_path.mkdir(parents=True, exist_ok=True)

    # Create Gocator output directory
    output_path = Path(GOCATOR_OUTPUT_DIR)
    goc_poses_path = output_path / "poses"
    scans_path = output_path / "scans"
    if output_path.exists():
        shutil.rmtree(output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    goc_poses_path.mkdir(parents=True, exist_ok=True)
    scans_path.mkdir(parents=True, exist_ok=True)

    camera.acquire_intrinsics(output_dir=ZEDX_OUTPUT_DIR)
    zed_poses = generator.generate_poses()

    robot.set_active_tcp("zedx")

    for i, pose in enumerate(zed_poses):
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
        save_pose_matrix(flange_matrix, pose_num=i, save_dir=str(zed_poses_path), file_prefix="pose_")

        # Capture RGB-D data
        camera.capture_rgbd()

    camera.download_captures(output_dir=ZEDX_OUTPUT_DIR)

    # Go back to the first ZEDX pose 
    robot.move_to_pose(
        x=zed_poses[0]['x'], 
        y=zed_poses[0]['y'], 
        z=zed_poses[0]['z'],
        rx=zed_poses[0]['rx'], 
        ry=zed_poses[0]['ry'], 
        rz=zed_poses[0]['rz'],
        frame="table"
    )

    print("ZEDX sequence completed. Processing coarse intersection data...")
    start_time = time.time()
    coarse_lines = coarse_predictor.run()
    end_time = time.time()
    print(f"Coarse line prediction took {end_time - start_time:.2f} seconds.")

    if coarse_lines is None:
        print("No coarse lines provided. Skipping Gocator sequence.")
        robot.go_home(speed=0.5, acc=0.5)
        return

    coarse_line_table = convert_points_base_to_table(coarse_lines, robot.table_conf)
    coarse_lines = [([coarse_line_table[0][0], coarse_line_table[0][1], coarse_line_table[0][2]],
                    [coarse_line_table[1][0], coarse_line_table[1][1], coarse_line_table[1][2]])]

    print(coarse_lines)

    # check if lines are on the table
    table_width = robot.table_conf.get('width', 0.510)
    table_height = robot.table_conf.get('height', 0.305)
    offset = 0.05  # 5 cm safety margin
    for line in coarse_lines:
        for p in line:
            if p[0] < -offset or p[0] > table_width + offset or p[1] < -offset or p[1] > table_height + offset or p[2] < -offset:
                print("SAFETY CHECK FAILED: Coarse line point is outside of the table boundaries. Aborting sequence.")
                robot.go_home(speed=0.5, acc=0.5)
                return

    print("GOCATOR CAMERA SEQUENCE")
    robot.set_active_tcp("gocator")

    global_pose_idx = 0

    for line_idx, line in enumerate(coarse_lines):

        print(f"Processing coarse line {line_idx}: {line}")

        # Init Gocator camera and pose generator
        generator = GocatorPoseGenerator(start_point=line[0],
                                         end_point=line[1],
                                         fov_width=FOV_WIDTH,
                                         overlap=0.1)

        goc_poses = generator.generate_poses()

        # Move to the first pose (approach position) to prevent collision
        # We use euler_rz directly if it is provided like in demo_gocator, or fallback to rz
        rz_approach = goc_poses[0].get('euler_rz', goc_poses[0]['rz'])
        robot.go_to_table_center(z=zed_poses[0]['z'], rz=rz_approach)

        for i, pose in enumerate(goc_poses):
            print(f"Moving to pose {global_pose_idx}")
            robot.move_to_pose(
                x=pose['x'],
                y=pose['y'],
                z=pose['z'],
                rx=pose.get('rx', 0.0),
                ry=pose['ry'],
                rz=pose['rz'],
                frame="table"
            )

            # Retrieve and save the flange pose matrix
            flange_matrix = get_flange_pose_matrix(robot.get_actual_q())
            save_pose_matrix(flange_matrix, pose_num=global_pose_idx, save_dir=str(goc_poses_path), file_prefix="pose_")

            # Capture scan data
            result = scanner.take_scan()
            if result["status"] == "success":
                scanner.save_scan(result["data"], pose_num=global_pose_idx, save_dir=str(scans_path), file_prefix="pose_", format="ply")
            else:
                print(f"Scan failed at pose {global_pose_idx}.")

            global_pose_idx += 1

    robot.go_to_table_center(z=zed_poses[0]['z'], rz=0.0)

    print("Gocator sequence completed. Processing fine intersection data...")

    # Merge point clouds and save output
    merger = PointCloudMerger(data_dir=GOCATOR_OUTPUT_DIR, hand_eye_file=Path("config/gocator_handeye.json"))
    merged_ply_path = merger.process()

    print("JOINT DETECTION ON MERGED POINT CLOUD")

    # Run joint detection on the merged point cloud
    joint_detector = JointDetectionModel(ply_file=str(merged_ply_path))
    joint_detector.process(filter_intersections=True, visualize_results=False)
    
    welding_line_points = np.asarray(joint_detector.intersection_data[0]['line_set'].points)
    welding_line_points_m = welding_line_points / 1000.0

    # Convert coordinates from base to table frame
    welding_line_points_table = convert_points_base_to_table(welding_line_points_m, robot.table_conf)

    # Sort points by x-coordinate to force left-to-right order
    welding_line_points_table = sorted(welding_line_points_table, key=lambda p: p[0])

    print("Welding line points (table frame):", welding_line_points_table)

    print("TORCH SEQUENCE")

    robot.set_active_tcp("torch")

    rz_angle = torch_angle_from_line(welding_line_points_table[0], welding_line_points_table[1])
    
    # Décalage selon l'orientation de la pièce
    offset_x = -0.05 if rz_angle > 0 else 0.05
    #offset_y = 0.1 if rz_angle > 0 else -0.1

    robot.go_to_table_center(z=zed_poses[0]['z'])
    robot.go_to_table_center(z=zed_poses[0]['z'], rz=rz_angle)

    robot.move_to_pose(
                x=welding_line_points_table[0][0] + offset_x, 
                #y=welding_line_points_table[0][1] + offset_y,
                y=welding_line_points_table[0][1] + 0.1,
                z=welding_line_points_table[0][2] + 0.1,
                rx=0.0,
                ry=0.0, 
                rz=rz_angle,
                frame="table"
            )

    robot.move_to_pose(
                x=welding_line_points_table[0][0], 
                y=welding_line_points_table[0][1] + TORCH_OFFSET_Y, 
                z=welding_line_points_table[0][2] + TORCH_OFFSET_Z,
                rx=-3.14159/4,
                ry=0.0, 
                rz=rz_angle,
                frame="table"
            )
    
    robot.move_to_pose(
                x=welding_line_points_table[1][0], 
                y=welding_line_points_table[1][1] + TORCH_OFFSET_Y, 
                z=welding_line_points_table[1][2] + TORCH_OFFSET_Z,
                rx=-3.14159/4,
                ry=0.0, 
                rz=rz_angle,
                frame="table",
                speed=0.02,
                acc=0.02
            )

    robot.go_home(speed=0.5, acc=0.5)

    print("Sequence completed.")


if __name__ == "__main__":
    main()