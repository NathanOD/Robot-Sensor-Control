import os
import sys
import shutil
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.zedx import ZedXCamera
from core.robot import RobotController
from core.pose_utils import get_flange_pose_matrix, save_pose_matrix

OUTPUT_DIR = "data_zedx"

def main():
	print("ZED-X SINGLE ACQUISITION")

	robot = RobotController(config_dir="config")
	camera = ZedXCamera(config_dir="config")

	output_path = Path(OUTPUT_DIR)
	poses_path = output_path / "poses"
	if output_path.exists():
		shutil.rmtree(output_path)
	poses_path.mkdir(parents=True, exist_ok=True)
	
	flange_matrix = get_flange_pose_matrix(robot.get_actual_q())
	save_pose_matrix(flange_matrix, pose_num=0, save_dir=str(poses_path), file_prefix="pose_")
    
	camera.capture_rgbd()
	camera.download_captures(output_dir=OUTPUT_DIR)


if __name__ == "__main__":
	main()
