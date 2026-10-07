import os
import sys
import shutil
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.robot import RobotController
from sensors.gocator import GocatorCamera
from core.pose_utils import get_flange_pose_matrix, save_pose_matrix

OUTPUT_DIR = "data_gocator"

def main():
	print("GOCATOR SINGLE ACQUISITION")

	robot = RobotController(config_dir="config")
	scanner = GocatorCamera(config_dir="config")

	output_path = Path(OUTPUT_DIR)
	poses_path = output_path / "poses"
	poses_sim_path = output_path / "poses_sim"
	scans_path = output_path / "scans"
	image_path = output_path / "images"
	if output_path.exists():
		shutil.rmtree(output_path)
	poses_path.mkdir(parents=True, exist_ok=True)
	poses_sim_path.mkdir(parents=True, exist_ok=True)
	scans_path.mkdir(parents=True, exist_ok=True)
	image_path.mkdir(parents=True, exist_ok=True)

	q = robot.get_actual_q()
	flange_matrix = get_flange_pose_matrix(q)
	save_pose_matrix(flange_matrix, pose_num=0, save_dir=str(poses_path), file_prefix="pose_")

	flange_matrix_sim = get_flange_pose_matrix(q, use_calibration=False)
	save_pose_matrix(flange_matrix_sim, pose_num=0, save_dir=str(poses_sim_path), file_prefix="pose_")

	result = scanner.take_scan()
	if result["status"] == "success":
		scanner.save_scan(result["data"], pose_num=0, save_dir=str(scans_path), file_prefix="pose_", format="ply")
		print("Acquisition completed. Data saved to:", OUTPUT_DIR)
	else:
		print("Acquisition failed.")

	#result_intensity = scanner.take_intensity_image()
	#if result_intensity["status"] == "success":
	#	scanner.save_intensity_image(result_intensity["data"], pose_num=0, save_dir=str(image_path), file_prefix="pose_")
	#	print("Intensity acquisition completed. Image saved to:", OUTPUT_DIR)
	#else:
	#	print("Intensity acquisition failed.")


if __name__ == "__main__":
    main()